/*
 * Lontium LT8911 LVDS to EDP driver
 *
 * Copyright  (C)  2016 - 2017 Topband. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the Lontium's LT8911 IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Author: shenhaibo
 * Version: 1.0.0
 * Release Date: 2018/2/28
 */


#ifndef _LONTIUM_LT8911_H_
#define _LONTIUM_LT8911_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/usb.h>
#include <linux/power_supply.h>

#define LT8911_I2C_NAME		"lt8911"
#define LT8911_DRIVER_VERSION  "1.0.0"

#define LT8911_ADDR_LENGTH      1
#define I2C_MAX_TRANSFER_SIZE   255
#define RETRY_MAX_TIMES		    3

/* LT8911 IC register define */
#define LT8911_REG_CHIP_ID_H	0x00
#define LT8911_REG_CHIP_ID_L	0x01

#define _LT8911_
//#define _LT8911B_

//#define _1080P_EDP_PANEL_
#define _1366x768_EDP_PANEL_
//#define _1600x900_EDP_PANEL_
//#define _1920x1200_EDP_PANEL_

/* Set the number of channels entered by LVDS. */
//#define _2PORT_LVDS_INPUT_
#define _1PORT_LVDS_INPUT_

#define _DE_MODE_
//#define _SYNC_MODE_

/* Output the Test pattern of black and white bars. */
//#define _TEST_PATTERN_ 

#define FRC_I2C_MASTER_INDEX 0

/* According to the eDP screen specification, 
the eDP screen in Lane 4 is generally 1.62G. */
#define _LANE_RATE_27G_TX_                    // 2.7G 
//define _LANE_RATE_16G_TX_                   // 1.62G

#define SWING_LEVEL_CLOSE 0x00

#define SWING_LEVEL_0_H 0x00
#define SWING_LEVEL_0_L 0xa0    //0xa0

#define SWING_LEVEL_1_H 0x00
#define SWING_LEVEL_1_L 0xf0    //0xf0

#define SWING_LEVEL_2_H 0x01
#define SWING_LEVEL_2_L 0x40

#define SWING_LEVEL_3_H 0x02
#define SWING_LEVEL_3_L 0xb4

#define _SSC_ 0x07

#ifdef _1080P_EDP_PANEL_

/* According to the Timing of the front-end LVDS signal, 
fill in the following array, and the LT8911 register 
will be set with this array. */
static int LVDS_timing[] =
/* H_act V_act H_total V_total H_BP H_sync V_sync V_BP */
{1920, 1080, 2200, 1125, 148, 44, 5, 36};         // 1080P Vesa Timing
//{1920, 1080, 2142, 1110, 80, 32, 6, 15};      // 1080P
//{1920, 1080, 2180, 1150, 180, 32, 3, 30};     // 1080P

/* According to the specification of the eDP screen, 
define the Lane number of LT8911 eDP output. */
#define _2_LANE_                                // eDP Output
//#define _4_LANE_                              // eDP Output

/* According to the color depth of the eDP screen, 
define the color depth Settings of LT8911. */
//#define _8_BIT_COLOR_DEPTH_                   // eDP panel Color Depth，16.7M color
#define _6_BIT_COLOR_DEPTH_                     // eDP panel Color Depth，262K color

#endif

#ifdef _1366x768_EDP_PANEL_

/* According to the Timing of the front-end LVDS signal, 
fill in the following array, and the LT8911 register 
will be set with this array. */
static int LVDS_timing[] =
/* H_act V_act H_total V_total H_BP H_sync V_sync V_BP */
{1366,  768,    1500,   800,    64, 56, 3,  28};// 1366x768 VESA Timing
//{ 1366, 768, 1592, 800, 104, 96, 3, 26 };     // N11BGE-E32 76.42MHz

/* According to the specification of the eDP screen, 
define the Lane number of LT8911 eDP output. */
#define _1_LANE_                                // eDP Output

/* According to the color depth of the eDP screen, 
define the color depth Settings of LT8911. */
#define _8_BIT_COLOR_DEPTH_                     // eDP panel Color Depth，16.7M color
//#define _6_BIT_COLOR_DEPTH_                   // eDP panel Color Depth，262K color

#endif

#ifdef _1600x900_EDP_PANEL_

/* According to the Timing of the front-end LVDS signal, 
fill in the following array, and the LT8911 register 
will be set with this array. */
static int LVDS_timing[] =
/* H_act V_act H_total V_total H_BP H_sync V_sync V_BP */
{ 1600, 900, 1940, 926, 260, 32,  5, 18 };        // 1366x768 for WK
//{ 1600, 900, 1798, 1112, 80, 38, 12, 100 };

/* According to the specification of the eDP screen, 
define the Lane number of LT8911 eDP output. */
#define _1_LANE_                                // eDP Output

/* According to the color depth of the eDP screen, 
define the color depth Settings of LT8911. */
//#define _8_BIT_COLOR_DEPTH_                   // eDP panel Color Depth，16.7M color
#define _6_BIT_COLOR_DEPTH_                     // eDP panel Color Depth，262K color

#endif

#ifdef _1920x1200_EDP_PANEL_

/* According to the Timing of the front-end LVDS signal, 
fill in the following array, and the LT8911 register 
will be set with this array. */
static int LVDS_timing[] =
/* H_act V_act H_total V_total H_BP H_sync V_sync V_BP */
{ 1920, 1200, 2080, 1235, 80, 32, 6, 26 };        // 1920x1200  154MHz Vesa Timing
//{1920,1200, 2592, 1245, 336, 200, 6, 36};     // 1920x1200  193MHz Vesa Timing

/* According to the specification of the eDP screen, 
define the Lane number of LT8911 eDP output. */
#define _2_LANE_                                // eDP Output

/* According to the color depth of the eDP screen, 
define the color depth Settings of LT8911. */
#define _8_BIT_COLOR_DEPTH_                     // eDP panel Color Depth，16.7M color
//#define _6_BIT_COLOR_DEPTH_                   // eDP panel Color Depth，262K color

#endif

enum {
    H_act = 0,
    V_act,
    H_tol,
    V_tol,
    H_bp,
    H_sync,
    V_sync,
    V_bp
};

struct lt8911_data {
	struct i2c_client *client;
	int pwr_gpio;
	int rst_gpio;
};

#endif /*_LONTIUM_LT8911_H_*/

