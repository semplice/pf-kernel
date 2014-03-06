/*
 * Support for rfkill for dritek based Acer Aspire and Travelmate series
 * notebooks.
 *
 * Copyright 2002-2007, Olaf Tauber <olaf-tauber@versanet.de>
 * Copyright 2004, Joachim Fenkes <acerhk@dojoe.net>
 * Copyright 2005, Antonio Cuni <cuni@programmazione.it>
 * Copyright 2009-2013, Stefan Lippers-Hollmann, <s.l-h@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef CONFIG_X86
#error This driver is only available for the x86 architecture
#endif

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/moduleparam.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/kmod.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/mc146818rtc.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/efi.h>
#include <linux/ioctl.h>
#include <linux/preempt.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#define ACERHK_MINOR MISC_DYNAMIC_MINOR

#define ACERHK_GET_KEYCOUNT      _IOR('p', 0x01, char)	/* Get number of cached key presses */
#define ACERHK_GET_KEYID         _IOR('p', 0x02, char)	/* Get first key in queue */
#define ACERHK_CONNECT           _IO('p', 0x03)	/* ? */
#define ACERHK_DISCONNECT        _IO('p', 0x04)	/* ? */
#define ACERHK_GET_THERMAL_EVENT _IOR('p', 0x05, short)	/* ? */
#define ACERHK_MAIL_LED_OFF      _IO('p', 0x10)	/* switch mail LED off */
#define ACERHK_MAIL_LED_ON       _IO('p', 0x11)	/* switch mail LED on */

/* all possible keys (known to me) */
typedef enum e_key_names {
	k_none = 0,
	k_help = 1,		/* Fn+F1 */
	k_setup = 2,		/* Fn+F2 */
	k_p1 = 3,
	k_p2 = 4,
	k_p3 = 5,
	k_www = 6,
	k_mail = 7,
	k_wireless = 8,
	k_power = 9,		/* Fn+F3 */
	k_mute = 10,		/* Fn+F8 */
	k_volup = 11,		/* Fn+Up */
	k_voldn = 12,		/* Fn+Down */
	k_res = 13,		/* resolution change on Medion MD 40100 */
	k_close = 14,		/* if lid is closed in tablet mode */
	k_open = 15,		/* if lid is opend in tablet mode */
	k_wireless2 = 16,	/* second wireless button on TM 243LC */
	k_play = 17,		/* Play/Pause found on AOpen */
	k_stop = 18,		/* Stop/Eject found on AOpen */
	k_prev = 19,		/* Prev found on AOpen */
	k_next = 20,		/* Next found on AOpen */
	k_display = 21		/* Change internal/external display on MD 42200 */
} t_key_names;
#define NR_KEY_NAMES 22
typedef unsigned int t_map_name2event[NR_KEY_NAMES];

/* available features */
#define TM_F_WLAN_EC1     0x00000010
#define TM_F_BLUE_EC1     0x00000020
#define TM_F_WLAN_EC2     0x00000040
#define TM_F_BLUE_EC2     0x00000080
#define TM_F_MUTE_LED_EC  0x00001000
#define TM_F_MAIL_LED     0x00010000
#define TM_F_MAIL_LED_EC  0x00020000
#define TM_F_MAIL_LED_EC2 0x00040000
#define TM_F_MAIL_LED_EC3 0x00080000

#define TM_F_CONNECT      0x00100000
#define TM_F_THERMAL      0x00200000
#define TM_F_PBUTTON      0x00400000
#define TM_F_WBUTTON      0x00800000

struct register_buffer {
	unsigned int eax;
	unsigned int ebx;
	unsigned int ecx;
	unsigned int edx;
	unsigned int edi;
	unsigned int esi;
	unsigned int ebp;
};

typedef asmlinkage void (*bios_call) (struct register_buffer *);

/* module parameters */
static int verbose = 1;
module_param(verbose, int, 0444);
MODULE_PARM_DESC(verbose, "output additional information");

/* input device */
static struct input_dev *acerhk_input_dev_ptr;

/* mapped IO area from 0xf0000 */
static void *reg1;

/* mapped IO area from 0xe0000 */
static void *reg2;

#ifndef __x86_64__
/* location of IO routine in mapped area */
static unsigned int bios_routine;

/* index of CMOS port to get key event */
static unsigned int cmos_index;
#endif

/* function for bios call */
static bios_call call_bios;

/* supported features for this model */
static unsigned int acerhk_model_features;

/* map of acer key codes to acer key names */
static unsigned char acerhk_key2name[0xff];

/* map of acer key names to key events */
static t_map_name2event acerhk_name2event;

/* wlan hardware toggle */
static int acerhk_wlan_state;

/* bluetooth hardware toggle */
static int acerhk_bluetooth_state;

/* {{{ Experimental use of dritek keyboard extension */
#define KBD_STATUS_REG		0x64	/* Status register (R) */
#define KBD_CNTL_REG		0x64	/* Controller command register (W) */
#define KBD_DATA_REG		0x60	/* Keyboard data register (R/W) */

static inline int my_i8042_read_status(void)
{
	return inb(KBD_STATUS_REG);
}

static int my_i8042_wait_write(void)
{
	int i = 0;

	while ((my_i8042_read_status() & 0x02) && (i < 10000)) {
		udelay(50);
		i++;
	}

	return -(i == 10000);
}

static void send_kbd_cmd(unsigned char cmd, unsigned char val)
{
	preempt_disable();
	if (!my_i8042_wait_write())
		outb(cmd, KBD_CNTL_REG);

	if (!my_i8042_wait_write())
		outb(val, KBD_DATA_REG);
	preempt_enable_no_resched();
}

static void enable_wlan_ec_1(void)
{
	send_kbd_cmd(0xe7, 0x01);
	acerhk_wlan_state = 1;
}

static void disable_wlan_ec_1(void)
{
	send_kbd_cmd(0xe7, 0x00);
	acerhk_wlan_state = 0;
}

static void enable_bluetooth_ec_1(void)
{
	send_kbd_cmd(0xe7, 0x03);
	acerhk_bluetooth_state = 1;
}

static void disable_bluetooth_ec_1(void)
{
	send_kbd_cmd(0xe7, 0x02);
	acerhk_bluetooth_state = 0;
}

static void enable_wlan_ec_2(void)
{
	send_kbd_cmd(0x45, acerhk_bluetooth_state ? 0xa2 : 0xa0);
	acerhk_wlan_state = 1;
}

static void disable_wlan_ec_2(void)
{
	send_kbd_cmd(0x45, acerhk_bluetooth_state ? 0xa1 : 0xa3);
	acerhk_wlan_state = 0;
}

static void enable_bluetooth_ec_2(void)
{
	send_kbd_cmd(0x45, acerhk_wlan_state ? 0xa2 : 0xa1);
	acerhk_bluetooth_state = 1;
}

static void disable_bluetooth_ec_2(void)
{
	send_kbd_cmd(0x45, acerhk_wlan_state ? 0xa0 : 0xa3);
	acerhk_bluetooth_state = 0;
}

static void enable_dritek_keyboard(void)
{
	send_kbd_cmd(0x59, 0x90);
}

static void disable_dritek_keyboard(void)
{
	send_kbd_cmd(0x59, 0x91);
}

static void enable_mail_led_ec_1(void)
{
	send_kbd_cmd(0xe8, 0x01);
}

static void disable_mail_led_ec_1(void)
{
	send_kbd_cmd(0xe8, 0x00);
}

static void enable_mail_led_ec_2(void)
{
	send_kbd_cmd(0x59, 0x92);
}

static void disable_mail_led_ec_2(void)
{
	send_kbd_cmd(0x59, 0x93);
}

static void enable_mail_led_ec_3(void)
{
	preempt_disable();
	if (!my_i8042_wait_write())
		outl(0x80008894, 0xCF8);

	if (!my_i8042_wait_write())
		outw(0xC061, 0xCFC);
	preempt_enable_no_resched();
}

static void disable_mail_led_ec_3(void)
{
	preempt_disable();
	if (!my_i8042_wait_write())
		outl(0x80008894, 0xCF8);

	if (!my_i8042_wait_write())
		outw(0xC060, 0xCFC);
	preempt_enable_no_resched();
}
/* Experimental use of dritek keyboard extension }}} */

/* {{{ hardware access functions */
/* call_bios_<model family>
 *
 * call request handler in mapped system rom
 *
 * the request is handed over via all 6 general purpose registers, results are
 * taken from them and copied back to buf
 */
static asmlinkage void call_bios_6xx(struct register_buffer *buf)
{
#ifndef __x86_64__
	if (bios_routine) {
		local_irq_disable();
		__asm__ __volatile__("movl %1,%%edx\n\t"
				     "pusha\n\t"
				     "movl %%edx,%%ebp\n\t"
				     "movl (%%ebp),%%eax\n\t"
				     "movl 4(%%ebp),%%ebx\n\t"
				     "movl 8(%%ebp),%%ecx\n\t"
				     "movl 12(%%ebp),%%edx\n\t"
				     "movl 16(%%ebp),%%edi\n\t"
				     "movl 20(%%ebp),%%esi\n\t"
				     "pushl %%ebp\n\t"
				     "call *%0\n\t"
				     "popl %%ebp\n\t"
				     "movl %%eax, (%%ebp)\n\t"
				     "movl %%ebx, 4(%%ebp)\n\t"
				     "movl %%ecx, 8(%%ebp)\n\t"
				     "movl %%edx, 12(%%ebp)\n\t"
				     "movl %%edi, 16(%%ebp)\n\t"
				     "movl %%esi, 20(%%ebp)\n\t"
				     "popa\n\t" : : "m"(bios_routine), "m"(buf)
				     : "%eax", "%ebx", "%ecx", "%edx", "%edi",
				     "%esi", "%ebp");
		local_irq_enable();
	}
#endif
}

/* get_fnkey_event
 *
 * gets the first (oldest) key id from the queue of events
 *
 * return value: id of key
 */
static int get_fnkey_event(void)
{
	struct register_buffer regs;
	regs.eax = 0x9610;
	regs.ebx = 0x61C;

	/* clear other registers, some models need this */
	regs.ecx = 0;
	regs.edx = 0;

	preempt_disable();
	call_bios(&regs);
	preempt_enable_no_resched();

	return regs.eax & 0xffff;
}

/* get_thermal_event
 *
 * does what?
 *
 * return value: event ?
 */
static int get_thermal_event(void)
{
	struct register_buffer regs;
	if (acerhk_model_features & TM_F_THERMAL) {
		regs.eax = 0x9612;
		regs.ebx = 0x12e;

		preempt_disable();
		call_bios(&regs);
		preempt_enable_no_resched();

		if (verbose >= 4)
			pr_info("thermal event = 0x%x\n", regs.eax);
	} else {
		regs.eax = 0x00;

		if (verbose >= 4)
			pr_info("thermal event not supported\n");
	}

	return regs.eax & 0xffff;
}


/* wbutton_fct_1
 *
 * turn on installed Bluetooth hardware together with the corresponding LED
 *
 * val: 0       turns off the LED
 *      1       turns the LED to green/blue
 */
static void wbutton_fct_1(int val)
{
	struct register_buffer regs;

	if (acerhk_model_features & TM_F_WBUTTON) {
		acerhk_bluetooth_state = val;
		regs.eax = 0x9610;
		regs.ebx = ((val & 0xff) << 8) | 0x34;

		preempt_disable();
		call_bios(&regs);
		preempt_enable_no_resched();

		if (verbose >= 4)
			pr_info("wbutton1 = 0x%x\n", regs.eax);
	} else {
		if (verbose >= 4)
			pr_info("wbutton function 1 not supported\n");
	}
}

/* wbutton_fct_2
 *
 * turn on installed WLAN hardware together with the corresponding LED
 *
 * val: 0       turns off the LED
 *      1       turns the LED to orange
 */
static void wbutton_fct_2(int val)
{
	struct register_buffer regs;

	if (acerhk_model_features & TM_F_WBUTTON) {
		acerhk_wlan_state = val;
		regs.eax = 0x9610;
		regs.ebx = ((val & 0xff) << 8) | 0x35;

		preempt_disable();
		call_bios(&regs);
		preempt_enable_no_resched();

		if (verbose >= 4)
			pr_info("wbutton2 = 0x%x\n", regs.eax);
	} else {
		if (verbose >= 4)
			pr_info("wbutton function 2 not supported\n");
	}
}

/* get_nr_events
 *
 * gets the number of cached events (keys pressed) in queue. Up to 31 events
 * are cached.
 *
 * return value: number of events in queue
 */
static int get_nr_events(void)
{
	unsigned long flags;
	unsigned char c = 0;

	spin_lock_irqsave(&rtc_lock, flags);
#ifndef __x86_64__
	if (cmos_index)
		c = CMOS_READ(cmos_index);
	else if (verbose >= 4)
		pr_info("get_nr_events - no valid cmos index set\n");
#endif
	spin_unlock_irqrestore(&rtc_lock, flags);

	return c;
}

/* set_mail_led
 *
 * change state of mail led
 *
 * val: 0 - switch led off
 *		1 - switch led on
 */
static void set_mail_led(int val)
{
	struct register_buffer regs;

	if (acerhk_model_features & TM_F_MAIL_LED) {
		regs.eax = 0x9610;
		regs.ebx = ((val & 0xff) << 8) | 0x31;

		preempt_disable();
		call_bios(&regs);
		preempt_enable_no_resched();

		if (verbose >= 4)
			pr_info("mail led set to = 0x%x\n", val);
	} else if (acerhk_model_features & TM_F_MAIL_LED_EC) {
		if (val == 1)
			enable_mail_led_ec_1();
		else if (val == 0)
			disable_mail_led_ec_1();
	} else if (acerhk_model_features & TM_F_MAIL_LED_EC2) {
		if (val == 1)
			enable_mail_led_ec_2();
		else if (val == 0)
			disable_mail_led_ec_2();
	} else if (acerhk_model_features & TM_F_MAIL_LED_EC3) {
		if (val == 1)
			enable_mail_led_ec_3();
		else if (val == 0)
			disable_mail_led_ec_3();
	} else {
		if (verbose >= 4)
			pr_info("mail led not supported\n");
	}
}

/* launch_connect
 *
 * does what?
 * val: 1 - only known value from windows driver
 */
static void launch_connect(int val)
{
	struct register_buffer regs;

	if (acerhk_model_features & TM_F_CONNECT) {
		regs.eax = 0x9610;
		regs.ebx = ((val & 0xff) << 8) | 0x2e;

		preempt_disable();
		call_bios(&regs);
		preempt_enable_no_resched();

		if (verbose >= 4)
			pr_info("connect(%d) = 0x%x\n", val, regs.eax);
	} else {
		if (verbose >= 4)
			pr_info("connect not supported\n");
	}
}
/* hardware access functions }}} */

/* {{{ hardware probing */
#ifndef __x86_64__
static unsigned long __init find_hk_area(void)
{
	long offset, sig = -1; /* offset to signature in io area */
	unsigned int fkt =  0;

	/* Look for signature, start at 0xf0000, search until 0xffff0 */
	for (offset = 0; offset < 0xfffd; offset += 16) {
		if (readl(reg1 + offset) == 0x30552142) {
			sig = offset;
			offset = 0xffff;
		}
	}

	if (sig < 0) {
		pr_warn("could not find request handler, possibly not all functions available\n");
	} else {
		/* compute location of bios routine */
		fkt = readl(reg1 + sig + 5);

		/* adjust fkt to address of mapped IO area */
		if (fkt >= 0xf0000)
			fkt = (unsigned long)reg1 + fkt - 0xf0000;
		else if (fkt >= 0xe0000)
			fkt = (unsigned long)reg1 + fkt - 0xe0000;
		else
			fkt = 0;
	}

	return fkt;
}
#endif

static void __init setup_model_features(void)
{
	/* set the correct bios call function according to type */
	call_bios = call_bios_6xx;

	if (verbose >= 3)
		pr_info("using call_bios_6xx mode\n");

	/* setup available keys, clear mapping keycode -> keyname, */
	memset(&acerhk_key2name[0], k_none, sizeof(acerhk_key2name));

	/* keys are handled by dritek EC */
	acerhk_key2name[1] = k_none;
	acerhk_key2name[2] = k_none;
}
/* hardware probing }}} */

/* {{{ key polling and translation */
#ifdef CONFIG_DEBUG_FS
static int acerhk_status_show(struct seq_file *m, void *v)
{
#ifndef __x86_64__
	if (bios_routine != 0) {
		seq_printf(m, "request handler: 0x%x\n", bios_routine);
		if (cmos_index) {
			seq_printf(m, "CMOS index:      0x%x\n", cmos_index);
			seq_printf(m, "events pending:  %u\n", get_nr_events());
		} else {
			seq_printf(m, "CMOS index:      not available\n");
		}
	} else {
		seq_printf(m, "request handler: not found\n");
	}
#else
	seq_printf(m, "Acer Travelmate hotkey driver dummy (x86_64).\n");
#endif

	return 0;
}

static int acerhk_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, acerhk_status_show, &inode->i_private);
}

static const struct file_operations acerhk_status_fops = {
	.open		= acerhk_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int acerhk_features_show(struct seq_file *m, void *v)
{
	if (acerhk_model_features & TM_F_MUTE_LED_EC)
		seq_printf(m, "mute led is supported\n");

	if (acerhk_model_features & TM_F_MAIL_LED)
		seq_printf(m, "mail led is supported\n");
	else if (acerhk_model_features & TM_F_MAIL_LED_EC)
		seq_printf(m, "mail led (EC) is supported\n");
	else if (acerhk_model_features & TM_F_MAIL_LED_EC2)
		seq_printf(m, "mail led (EC2) is supported\n");
	else if (acerhk_model_features & TM_F_MAIL_LED_EC3)
		seq_printf(m, "mail led (EC3) is supported\n");

	if (acerhk_model_features & TM_F_WLAN_EC1)
		seq_printf(m, "wlan control (EC1) is supported\n");
	else if (acerhk_model_features & TM_F_WLAN_EC2)
		seq_printf(m, "wlan control (EC2) is supported\n");

	if (acerhk_model_features & TM_F_BLUE_EC1)
		seq_printf(m, "bluetooth control (EC1) is supported\n");
	else if (acerhk_model_features & TM_F_BLUE_EC2)
		seq_printf(m, "bluetooth control (EC2) is supported\n");

	if (acerhk_model_features & TM_F_CONNECT)
		seq_printf(m, "supported function: connect");

	if (acerhk_model_features & TM_F_THERMAL)
		seq_printf(m, "supported function: thermal");

	if (acerhk_model_features & TM_F_PBUTTON)
		seq_printf(m, "supported function: pbutton");

	if (acerhk_model_features & TM_F_WBUTTON)
		seq_printf(m, "supported function: wbutton");

	return 0;
}

static int acerhk_features_open(struct inode *inode, struct file *file)
{
	return single_open(file, acerhk_features_show, &inode->i_private);
}

static const struct file_operations acerhk_features_fops = {
	.open		= acerhk_features_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int acerhk_supported_keys_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < 255; i++) {
		switch (acerhk_key2name[i]) {
		case k_help:
			seq_printf(m, "help\n");
			break;
		case k_setup:
			seq_printf(m, "setup\n");
			break;
		case k_p1:
			seq_printf(m, "p1\n");
			break;
		case k_p2:
			seq_printf(m, "p2\n");
			break;
		case k_p3:
			seq_printf(m, "p3\n");
			break;
		case k_www:
			seq_printf(m, "www\n");
			break;
		case k_mail:
			seq_printf(m, "mail\n");
			break;
		case k_wireless:
			seq_printf(m, "wireless\n");
			break;
		case k_power:
			seq_printf(m, "power\n");
			break;
		case k_mute:
			seq_printf(m, "mute\n");
			break;
		case k_volup:
			seq_printf(m, "volup\n");
			break;
		case k_voldn:
			seq_printf(m, "voldn\n");
			break;
		case k_res:
			seq_printf(m, "res\n");
			break;
		case k_close:
			seq_printf(m, "close\n");
			break;
		case k_open:
			seq_printf(m, "open\n");
			break;
		case k_wireless2:
			seq_printf(m, "wireless2\n");
			break;
		case k_play:
			seq_printf(m, "play\n");
			break;
		case k_stop:
			seq_printf(m, "stop\n");
			break;
		case k_prev:
			seq_printf(m, "prev\n");
			break;
		case k_next:
			seq_printf(m, "next\n");
			break;
		case k_display:
			seq_printf(m, "display\n");
			break;
		default:
			break;
		}
	}

	return 0;
}

static int acerhk_supported_keys_open(struct inode *inode, struct file *file)
{
	return single_open(file, acerhk_supported_keys_show, &inode->i_private);
}

static const struct file_operations acerhk_supported_keys_fops = {
	.open		= acerhk_supported_keys_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int acerhk_keymap_show(struct seq_file *m, void *v)
{
	seq_printf(m,
		   "help:      0x%x\n"
		   "setup:     0x%x\n"
		   "p1:        0x%x\n"
		   "p2:        0x%x\n"
		   "p3:        0x%x\n"
		   "www:       0x%x\n"
		   "mail:      0x%x\n"
		   "wireless:  0x%x\n"
		   "power:     0x%x\n"
		   "mute:      0x%x\n"
		   "volup:     0x%x\n"
		   "voldn:     0x%x\n"
		   "res:       0x%x\n"
		   "close:     0x%x\n"
		   "open:      0x%x\n"
		   "wireless2: 0x%x\n"
		   "play:      0x%x\n"
		   "stop:      0x%x\n"
		   "prev:      0x%x\n"
		   "next:      0x%x\n"
		   "display:   0x%x\n",
		   acerhk_name2event[k_help],
		   acerhk_name2event[k_setup],
		   acerhk_name2event[k_p1],
		   acerhk_name2event[k_p2],
		   acerhk_name2event[k_p3],
		   acerhk_name2event[k_www],
		   acerhk_name2event[k_mail],
		   acerhk_name2event[k_wireless],
		   acerhk_name2event[k_power],
		   acerhk_name2event[k_mute],
		   acerhk_name2event[k_volup],
		   acerhk_name2event[k_voldn],
		   acerhk_name2event[k_res],
		   acerhk_name2event[k_close],
		   acerhk_name2event[k_open],
		   acerhk_name2event[k_wireless2],
		   acerhk_name2event[k_play],
		   acerhk_name2event[k_stop],
		   acerhk_name2event[k_prev],
		   acerhk_name2event[k_next],
		   acerhk_name2event[k_display]);

	return 0;
}

static int acerhk_keymap_open(struct inode *inode, struct file *file)
{
	return single_open(file, acerhk_keymap_show, &inode->i_private);
}

static const struct file_operations acerhk_keymap_fops = {
	.open		= acerhk_keymap_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static void acerhk_init_debugfs(void)
{
	struct dentry *d;

	d = debugfs_create_dir("acerhk", NULL);
	if (!d)
		return;

	(void) debugfs_create_file("status", 0444, d, NULL, &acerhk_status_fops);
	(void) debugfs_create_file("features", 0444, d, NULL, &acerhk_features_fops);
	(void) debugfs_create_file("supported_keys", 0444, d, NULL, &acerhk_supported_keys_fops);
	(void) debugfs_create_file("keymap", 0444, d, NULL, &acerhk_keymap_fops);
}
#endif /* CONFIG_DEBUG_FS */

static void set_keymap_name(t_key_names name, unsigned int key)
{
	acerhk_name2event[name] = key;
}

static void init_keymap_input(void)
{
	/* these values for input keys are chosen to match the key names on the
	   actual Acer laptop */
	set_keymap_name(k_none, KEY_RESERVED);
	set_keymap_name(k_help, KEY_HELP);
	set_keymap_name(k_setup, KEY_CONFIG);
	set_keymap_name(k_p1, KEY_PROG1);
	set_keymap_name(k_p2, KEY_PROG2);
	set_keymap_name(k_p3, KEY_PROG3);
	set_keymap_name(k_www, KEY_WWW);
	set_keymap_name(k_mail, KEY_MAIL);
	set_keymap_name(k_wireless, KEY_XFER);
	set_keymap_name(k_power, KEY_POWER);
	set_keymap_name(k_mute, KEY_MUTE);
	set_keymap_name(k_volup, KEY_VOLUMEUP);
	set_keymap_name(k_voldn, KEY_VOLUMEDOWN);
	set_keymap_name(k_res, KEY_CONFIG);
	set_keymap_name(k_close, KEY_CLOSE);
	set_keymap_name(k_open, KEY_OPEN);
	/* I am not really happy with the selections for wireless and wireless2,
	   but coffee looks good. Michal Veselenyi proposed this value */
	set_keymap_name(k_wireless2, KEY_COFFEE);
	set_keymap_name(k_play, KEY_PLAYPAUSE);
	set_keymap_name(k_stop, KEY_STOPCD);
	set_keymap_name(k_prev, KEY_PREVIOUSSONG);
	set_keymap_name(k_next, KEY_NEXTSONG);
	set_keymap_name(k_display, KEY_MEDIA);	/* also not happy with this */
}

static int init_input(void)
{
	int i;
	int ret;

	/* allocate acerhk input device */
	acerhk_input_dev_ptr = input_allocate_device();

	/* enter some name */
	acerhk_input_dev_ptr->name = "Acer hotkey driver";

	/* some laptops have a mail led, should I announce it here? */
	acerhk_input_dev_ptr->evbit[0] = BIT(EV_KEY);

	/* announce keys to input system
	 * the generated keys can be changed on runtime,
	 * but to publish those changes the device needs to
	 * get reconnected (I dont't know any other way)
	 * Therefore I enable all possible keys */
	for (i = KEY_RESERVED; i < BTN_MISC; i++)
		set_bit(i, acerhk_input_dev_ptr->keybit);

	/* set mapping keyname -> input event */
	init_keymap_input();
	ret = input_register_device(acerhk_input_dev_ptr);
	if (ret) {
		pr_err("failed to register input device: %d\n", ret);
		input_free_device(acerhk_input_dev_ptr);
	}

	return ret;
}

static void release_input(void)
{
	input_unregister_device(acerhk_input_dev_ptr);
}
/* key polling and translation }}} */


/* {{{ file operations */
static long acerhk_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	int retval;
	switch (cmd) {
	case ACERHK_GET_KEYCOUNT:
		{
			char nr;
			nr = get_nr_events();
			put_user(nr, (char *)arg);
			retval = 0;
			break;
		}
	case ACERHK_GET_KEYID:
		{
			char id;
			id = get_fnkey_event();
			put_user(id, (char *)arg);
			retval = 0;
			break;
		}
	case ACERHK_CONNECT:
		launch_connect(1);
		retval = 0;
		break;
	case ACERHK_DISCONNECT:
		launch_connect(0);
		retval = 0;
		break;
	case ACERHK_GET_THERMAL_EVENT:
		{
			short event;
			event = get_thermal_event();
			put_user(event, (short *)arg);
			retval = 0;
			break;
		}
	case ACERHK_MAIL_LED_OFF:
		set_mail_led(0);
		retval = 0;
		break;
	case ACERHK_MAIL_LED_ON:
		set_mail_led(1);
		retval = 0;
		break;
	default:
		retval = -EINVAL;
	}
	return retval;
}

static int acerhk_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int acerhk_release(struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef CONFIG_PM
static int acerhk_resume(struct platform_device *dev)
{
	pr_info("Resuming. Setting wlan_state to: %d\n",
		acerhk_wlan_state);

	if (acerhk_wlan_state)
		wbutton_fct_2(1);
	else
		wbutton_fct_2(0);

	return 0;
}
#endif

static const struct file_operations acerhk_fops = {
owner:		  THIS_MODULE,
unlocked_ioctl	: acerhk_unlocked_ioctl,
open		: acerhk_open,
release		: acerhk_release,
};

static struct miscdevice acerhk_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KBUILD_MODNAME,
	.fops = &acerhk_fops,
};
/* file operations }}} */

#ifndef __x86_64__
static void model_init(void)
{
	/* set callroutine, features and keymap for model */
	setup_model_features();

	/* Launch connect only if available */
	if (acerhk_model_features & TM_F_CONNECT) {
		if (verbose)
			pr_info("Model type dritek, calling launch_connect(1)\n");

		launch_connect(1);
	}
	enable_dritek_keyboard();
}
#endif

static void __init dmi_check_cb_common(const struct dmi_system_id *id, bool untested)
{
	pr_info("Identified notebook model: '%s'\n", id->ident);

	if (untested)
		pr_warn("Untested device, please report success or failure to the module maintainer.\n");
}

static int __init dmi_check_cb_aspire_1300(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, wireless hardware can be controlled */
	acerhk_model_features = 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_aspire_1310(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, wireless hardware can be controlled */
	acerhk_model_features = 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_aspire_1350(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* mail led, handled by EC, wireless HW is not (yet) controllable? */
	acerhk_model_features = TM_F_MAIL_LED_EC | TM_F_WLAN_EC1;

	return 1;
}

static int __init dmi_check_cb_aspire_1360(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* mail led, handled by EC, wireless HW is not (yet) controllable? */
	acerhk_model_features = TM_F_MAIL_LED_EC | TM_F_WLAN_EC1;

	return 1;
}

static int __init dmi_check_cb_aspire_1400(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, wireless hardware can be controlled */
	acerhk_model_features = 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_aspire_1450(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Bluetooth/Wlan led, Mail led handled by EC (variant 3) */
	acerhk_model_features = TM_F_MAIL_LED_EC3 | TM_F_WBUTTON;

	return 1;
}

static int __init dmi_check_cb_aspire_1800(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Dritek EC, bluetooth, wifi, mail */
	acerhk_model_features = TM_F_MAIL_LED_EC2 | TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	/* Default state is on */
	acerhk_wlan_state = 1;

	return 1;
}

static int __init dmi_check_cb_aspire_2000(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* No features (?) dritek EC, mail LED is handled by EC but
	   different from other Aspire series */
	acerhk_model_features = TM_F_MAIL_LED_EC2;

	return 1;
}

static int __init dmi_check_cb_aspire_2010(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Dritek EC, bluetooth, wifi, mail */
	acerhk_model_features = TM_F_MAIL_LED_EC2 | TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	/* Default state is on */
	acerhk_wlan_state = 1;

	return 1;
}

static int __init dmi_check_cb_aspire_2020(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Dritek EC, bluetooth, wifi, mail */
	acerhk_model_features = TM_F_MAIL_LED_EC2 | TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	/* Default state is on */
	acerhk_wlan_state = 1;

	return 1;
}

static int __init dmi_check_cb_aspire_4150(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Dritek EC, bluetooth, wifi, mail */
	acerhk_model_features = TM_F_MAIL_LED_EC2 | TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	return 1;
}

static int __init dmi_check_cb_aspire_4650(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* Dritek EC, bluetooth, wifi, mail */
	/* According to Andreas Stumpfl his TM 4652LMi does also work
	 * as series 3200, which might mean that the BIOS function
	 * accesses the EC */
	acerhk_model_features = TM_F_MAIL_LED_EC2 | TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_290(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, false);

	/* no special functions, wireless hardware controlled by EC */
	acerhk_model_features = TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_420(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all functions and dritek EC, mail LED is handled by EC,
	 * second variant. An additional led is available, mute.
	 * (really?)
	 */
	acerhk_model_features = TM_F_MUTE_LED_EC | TM_F_MAIL_LED_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_430(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all functions and dritek EC, mail LED is handled by EC,
	 * second variant. An additional led is available, mute.
	 * (really?)
	 */
	acerhk_model_features = TM_F_MUTE_LED_EC | TM_F_MAIL_LED_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_530(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* No features(?) dritek EC, mail LED is handled by EC but
	   different from other Aspire series */
	acerhk_model_features = TM_F_MAIL_LED_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_540(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* No features(?) dritek EC, mail LED is handled by EC but
	   different from other Aspire series */
	acerhk_model_features = TM_F_MAIL_LED_EC2;

	return 1;
}

static int __init dmi_check_cb_travelmate_650(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, wireless hardware can be controlled */
	acerhk_model_features = 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_travelmate_660(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, mail led */
	acerhk_model_features = TM_F_MAIL_LED | 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_travelmate_800(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, mail led */
	acerhk_model_features = TM_F_MAIL_LED | 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_travelmate_1700(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* all special functions, wireless hardware can be controlled */
	acerhk_model_features = 0x00f00000;

	return 1;
}

static int __init dmi_check_cb_travelmate_2300(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* wireless hardware, hopefully under control of my driver */
	acerhk_model_features = TM_F_BLUE_EC1 | TM_F_WLAN_EC1;

	return 1;
}

static int __init dmi_check_cb_travelmate_2350(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* no special functions, wireless hardware controlled by EC */
	acerhk_model_features = TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	/* Default state is on */
	acerhk_wlan_state = 1;

	return 1;
}

static int __init dmi_check_cb_travelmate_3200(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* test, if this model uses old style wlan control */
	acerhk_model_features = TM_F_WBUTTON;

	return 1;
}

static int __init dmi_check_cb_travelmate_4000(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* wireless hardware, hopefully under control of my driver */
	acerhk_model_features = TM_F_BLUE_EC1 | TM_F_WLAN_EC1;

	return 1;
}

static int __init dmi_check_cb_travelmate_4050(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* no special functions, wireless hardware controlled by EC */
	acerhk_model_features = TM_F_WLAN_EC2 | TM_F_BLUE_EC2;

	/* Default state is on */
	acerhk_wlan_state = 1;

	return 1;
}

static int __init dmi_check_cb_travelmate_4500(const struct dmi_system_id *id)
{
	dmi_check_cb_common(id, true);

	/* wireless hardware, hopefully under control of my driver */
	acerhk_model_features = TM_F_BLUE_EC1 | TM_F_WLAN_EC1;

	return 1;
}

static const struct dmi_system_id acerhk_id_table[] = {
	{
		.ident = "Acer Aspire 1300",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1300"),
		},
		.callback = dmi_check_cb_aspire_1300
	},
	{
		.ident = "Acer Aspire 1310",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1310"),
		},
		.callback = dmi_check_cb_aspire_1310
	},
	{
		.ident = "Acer Aspire 1350",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1350"),
		},
		.callback = dmi_check_cb_aspire_1350
	},
	{
		.ident = "Acer Aspire 1360",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1360"),
		},
		.callback = dmi_check_cb_aspire_1360
	},
	{
		.ident = "Acer Aspire 1400",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1400"),
		},
		.callback = dmi_check_cb_aspire_1400
	},
	{
		.ident = "Acer Aspire 1450",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1450"),
		},
		.callback = dmi_check_cb_aspire_1450
	},
	{
		.ident = "Acer Aspire 1800",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 1800"),
		},
		.callback = dmi_check_cb_aspire_1800
	},
	{
		.ident = "Acer Aspire 2000",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 2000"),
		},
		.callback = dmi_check_cb_aspire_2000
	},
	{
		.ident = "Acer Aspire 2010",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 2010"),
		},
		.callback = dmi_check_cb_aspire_2010
	},
	{
		.ident = "Acer Aspire 2020",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 2020"),
		},
		.callback = dmi_check_cb_aspire_2020
	},
	{
		.ident = "Acer Aspire 4150",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 4150"),
		},
		.callback = dmi_check_cb_aspire_4150
	},
	{
		.ident = "Acer Aspire 4650",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Aspire 4650"),
		},
		.callback = dmi_check_cb_aspire_4650
	},
	{
		.callback = dmi_check_cb_travelmate_290,
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 290"),
		},
		.ident = "Acer TravelMate 290"
	},
	{
		.ident = "Acer TravelMate 420",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 420"),
		},
		.callback = dmi_check_cb_travelmate_420
	},
	{
		.ident = "Acer TravelMate 430",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 430"),
		},
		.callback = dmi_check_cb_travelmate_430
	},
	{
		.ident = "Acer TravelMate 530",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 530"),
		},
		.callback = dmi_check_cb_travelmate_530
	},
	{
		.ident = "Acer TravelMate 540",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 540"),
		},
		.callback = dmi_check_cb_travelmate_540
	},
	{
		.ident = "Acer TravelMate 650",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 650"),
		},
		.callback = dmi_check_cb_travelmate_650
	},
	{
		.ident = "Acer TravelMate 660",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 660"),
		},
		.callback = dmi_check_cb_travelmate_660
	},
	{
		.ident = "Acer TravelMate 800",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 800"),
		},
		.callback = dmi_check_cb_travelmate_800
	},
	{
		.ident = "Acer TravelMate 1700",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 1700"),
		},
		.callback = dmi_check_cb_travelmate_1700
	},
	{
		.ident = "Acer TravelMate 2300",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 2300"),
		},
		.callback = dmi_check_cb_travelmate_2300
	},
	{
		.ident = "Acer TravelMate 2350",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 2350"),
		},
		.callback = dmi_check_cb_travelmate_2350
	},
	{
		.ident = "Acer TravelMate 3200",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 3200"),
		},
		.callback = dmi_check_cb_travelmate_3200
	},
	{
		.ident = "Acer TravelMate 4000",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 4000"),
		},
		.callback = dmi_check_cb_travelmate_4000
	},
	{
		.ident = "Acer TravelMate 4050",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 4050"),
		},
		.callback = dmi_check_cb_travelmate_4050
	},
	{
		.ident = "Acer TravelMate 4500",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TravelMate 4500"),
		},
		.callback = dmi_check_cb_travelmate_4500
	},
	{ NULL, }
};

static int acerhk_probe(struct platform_device *dev)
{
	int ret;
	const struct dmi_system_id *system_id = dmi_first_match(acerhk_id_table);

	if (!system_id)
		return -ENXIO;

	ret = misc_register(&acerhk_misc_dev);
	if (ret) {
		pr_err("can't misc_register on minor=%d\n", ACERHK_MINOR);
		ret = -EAGAIN;
	} else {
		reg1 = ioremap(0xf0000, 0xffff);
		if (verbose >= 2)
			pr_info("area from 0xf000 to 0xffff mapped to %p\n",
				reg1);

		reg2 = ioremap(0xe0000, 0xffff);
		if (verbose >= 2)
			pr_info("area from 0xe000 to 0xffff mapped to %p\n",
				reg2);

		/* attach to input system */
		ret = init_input();
#ifndef __x86_64__
		bios_routine = find_hk_area();

		/* do model specific initialization */
		model_init();
#else
		setup_model_features();
		enable_dritek_keyboard();
#endif

#ifdef CONFIG_DEBUG_FS
		acerhk_init_debugfs();
#endif

		/* enable wlan LED */
		if (acerhk_model_features & TM_F_WLAN_EC1)
			enable_wlan_ec_1();
		else if (acerhk_model_features & TM_F_WLAN_EC2)
			enable_wlan_ec_2();
		else
			wbutton_fct_2(1);

		/* enable bluetooth LED */
		if (acerhk_model_features & TM_F_BLUE_EC1)
			enable_bluetooth_ec_1();
		else if (acerhk_model_features & TM_F_BLUE_EC2)
			enable_bluetooth_ec_2();
		else
			wbutton_fct_1(1);

		/* enable mail LED */
		set_mail_led(1);
	}

	return ret;
}

static int acerhk_remove(struct platform_device *dev)
{
	/* disable wireless LED */
	if (acerhk_model_features & TM_F_WLAN_EC1)
		disable_wlan_ec_1();
	else if (acerhk_model_features & TM_F_WLAN_EC2)
		disable_wlan_ec_2();
	else
		wbutton_fct_2(0);

	/* disable bluetooth LED */
	if (acerhk_model_features & TM_F_BLUE_EC1)
		disable_bluetooth_ec_1();
	else if (acerhk_model_features & TM_F_BLUE_EC2)
		disable_bluetooth_ec_2();
	else
		wbutton_fct_1(0);

	/* disable mail LED */
	set_mail_led(0);

	if (reg1)
		iounmap(reg1);

	if (reg2)
		iounmap(reg2);

	release_input();
	misc_deregister(&acerhk_misc_dev);
	disable_dritek_keyboard();

	return 0;
}

static struct platform_driver acerhk_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
	},
	.probe = acerhk_probe,
	.remove = acerhk_remove,
#ifdef CONFIG_PM
	.resume = acerhk_resume,
#endif
};

static struct platform_device *acerhk_platform_device;

static int __init acerhk_init(void)
{
	int error;

	if (efi_enabled(EFI_BOOT))
		return -ENODEV;

	if (!dmi_check_system(acerhk_id_table)) {
		pr_err("notebook not recognized, refusing to load module.\n");
		return -ENODEV;
	}

	error = platform_driver_register(&acerhk_driver);
	if (error)
		return error;

	acerhk_platform_device = platform_device_alloc("acerhk", -1);
	if (!acerhk_platform_device) {
		error = -ENOMEM;
		goto err_driver_unregister;
	}

	error = platform_device_add(acerhk_platform_device);
	if (error)
		goto err_free_device;

	return 0;

err_free_device:
	platform_device_put(acerhk_platform_device);
err_driver_unregister:
	platform_driver_unregister(&acerhk_driver);

	return error;
}

static void __exit acerhk_exit(void)
{
	platform_device_unregister(acerhk_platform_device);
	platform_driver_unregister(&acerhk_driver);
}

module_init(acerhk_init);
module_exit(acerhk_exit);

MODULE_AUTHOR("Olaf Tauber");
MODULE_AUTHOR("Stefan Lippers-Hollmann <s.l-h@gmx.de>");
MODULE_DESCRIPTION("AcerHotkeys extra buttons keyboard driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(dmi, acerhk_id_table);
