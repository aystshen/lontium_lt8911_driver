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
 * Notes:
 * 1. IIC address of LT8911:
 *   A) If LT8911's number 13 (S_ADR) is low, the I2C address of LT8911 is 0x52, 
 *      and bit0 is the read-write mark bit.
 *      If it is a Linux system, the I2C address of LT8911 is 0x29, and bit7 is 
 *      the read and write flag bit.
 *   B) If LT8911's 13th foot (S_ADR) is high, then LT8911's I2C address is 0x5a, 
 *      and bit0 is the read-write mark bit.
 *      If it is a Linux system, the I2C address of LT8911 is 0x2d, and bit7 is 
 *      the read and write flag bit.
 * 2. The IIC rate should not exceed 100KHz. 
 * 3. To ensure that LVDS signal is given to LT8911, then initialize LT8911. 
 * 4. The front-end master control GPIO is required to reply to LT8911. Before 
 *    the register, the LT8911 is reset.
 *    Use GPIO to lower LT8911's reset foot 100ms, then pull up and maintain 100ms.
 *
 * Author: shenhaibo
 * Version: 1.0.0
 * Release Date: 2018/2/28
 */

#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include "lt8911.h"

static u8 DPCD0000H;
static u8 DPCD0001H;
static u8 DPCD0002H;
static u8 DPCD0003H;
static u8 DPCD0004H;
static u8 DPCD0005H;
static u8 DPCD0006H;
static u8 DPCD0007H;
static u8 DPCD0008H;
static u8 DPCD0009H;
static u8 DPCD000aH;
static u8 DPCD000bH;
static u8 DPCD0200H;
static u8 DPCD0201H;
static u8 DPCD0202H;
static u8 DPCD0203H;
static u8 DPCD0204H;
static u8 DPCD0205H;
static u8 DPCD0206H;
#ifdef _4_LANE_
static u8 DPCD0207H;
#endif

static u8 swing_req = 0x00;

/*******************************************************
 * Function:
 *  Write data to the i2c slave device.
 * Input:
 *  client: i2c device.
 *  buf[0]: write start address.
 *  buf[1~len-1]: data buffer
 *  len: LT8911_ADDR_LENGTH + write bytes count
 * Output:
 *  numbers of i2c_msgs to transfer:
 *      0: succeed, otherwise: failed
 *********************************************************/
int lt8911_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
    unsigned int pos = 0, transfer_length = 0;
    u8 address = buf[0];
    unsigned char put_buf[64];
    int retry, ret = 0;
    struct i2c_msg msg = {
        .addr = client->addr,
        .flags = !I2C_M_RD,
    };

    if (likely(len < sizeof(put_buf))) {
        /* code optimize,use stack memory*/
        msg.buf = &put_buf[0];
    } else {
        msg.buf = kmalloc(len > I2C_MAX_TRANSFER_SIZE
                          ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
        if (!msg.buf)
            return -ENOMEM;
    }

    len -= LT8911_ADDR_LENGTH;
    while (pos != len) {
        if (unlikely(len - pos > I2C_MAX_TRANSFER_SIZE - LT8911_ADDR_LENGTH))
            transfer_length = I2C_MAX_TRANSFER_SIZE - LT8911_ADDR_LENGTH;
        else
            transfer_length = len - pos;
        msg.buf[0] = address;
        msg.len = transfer_length + LT8911_ADDR_LENGTH;
        memcpy(&msg.buf[LT8911_ADDR_LENGTH], &buf[LT8911_ADDR_LENGTH + pos], transfer_length);
        for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            if (likely(i2c_transfer(client->adapter, &msg, 1) == 1)) {
                pos += transfer_length;
                address += transfer_length;
                break;
            }
            dev_info(&client->dev, "I2C write retry[%d]\n", retry + 1);
            udelay(2000);
        }
        if (unlikely(retry == RETRY_MAX_TIMES)) {
            dev_err(&client->dev,
                    "I2c write failed,dev:%02x,reg:%02x,size:%u\n",
                    client->addr, address, len);
            ret = -EAGAIN;
            goto write_exit;
        }
    }
write_exit:
    if (len + LT8911_ADDR_LENGTH >= sizeof(put_buf))
        kfree(msg.buf);
    return ret;
}

/*******************************************************
 * Function:
 *  Read data from the i2c slave device.
 * Input:
 *  client: i2c device.
 *  buf[0]: read start address.
 *  buf[1~len-1]: data buffer
 *  len: LT8911_ADDR_LENGTH + read bytes count
 * Output:
 *  numbers of i2c_msgs to transfer:
 *      0: succeed, otherwise: failed
 *********************************************************/
int lt8911_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
    unsigned int transfer_length = 0;
    unsigned int pos = 0;
    u8 address = buf[0];
    unsigned char get_buf[64], addr_buf[2];
    int retry, ret = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = !I2C_M_RD,
            .buf = &addr_buf[0],
            .len = LT8911_ADDR_LENGTH,
        }, {
            .addr = client->addr,
            .flags = I2C_M_RD,
        }
    };

    len -= LT8911_ADDR_LENGTH;
    if (likely(len < sizeof(get_buf))) {
        /* code optimize, use stack memory */
        msgs[1].buf = &get_buf[0];
    } else {
        msgs[1].buf = kzalloc(len > I2C_MAX_TRANSFER_SIZE
                              ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
        if (!msgs[1].buf)
            return -ENOMEM;
    }

    while (pos != len) {
        if (unlikely(len - pos > I2C_MAX_TRANSFER_SIZE))
            transfer_length = I2C_MAX_TRANSFER_SIZE;
        else
            transfer_length = len - pos;
        msgs[0].buf[0] = address;
        msgs[1].len = transfer_length;
        for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            if (likely(i2c_transfer(client->adapter, msgs, 2) == 2)) {
                memcpy(&buf[LT8911_ADDR_LENGTH + pos], msgs[1].buf, transfer_length);
                pos += transfer_length;
                address += transfer_length;
                break;
            }
            dev_info(&client->dev, "I2c read retry[%d]:0x%x\n",
                     retry + 1, address);
            udelay(2000);
        }
        if (unlikely(retry == RETRY_MAX_TIMES)) {
            dev_err(&client->dev,
                    "I2c read failed,dev:%02x,reg:%02x,size:%u\n",
                    client->addr, address, len);
            ret = -EAGAIN;
            goto read_exit;
        }
    }
read_exit:
    if (len >= sizeof(get_buf))
        kfree(msgs[1].buf);
    return ret;
}

int lt8911_write(struct i2c_client *client, u8 addr, u8 data)
{
    u8 buf[2] = {addr, data};
    int ret = -1;

    ret = lt8911_i2c_write(client, buf, 2);

    return ret;
}

u8 lt8911_read(struct i2c_client *client, u8 addr)
{
    u8 buf[2] = {addr};
    int ret = -1;

    ret = lt8911_i2c_read(client, buf, 2);
    if (ret == 0) {
        return buf[1];
    } else {
        return 0;
    }
}

void lt8911_reset_guitar(struct i2c_client *client)
{
    struct lt8911_data *lt8911 = i2c_get_clientdata(client);

    dev_info(&client->dev, "Guitar reset");
    if (!gpio_is_valid(lt8911->rst_gpio)) {
        dev_warn(&client->dev, "reset failed no valid reset gpio");
        return;
    }

    gpio_direction_output(lt8911->rst_gpio, 0);

    usleep_range(100*1000, 150*1000); //150ms
    gpio_direction_output(lt8911->rst_gpio, 1);

    usleep_range(100*1000, 150*1000); //150ms
    gpio_direction_input(lt8911->rst_gpio);
}


static int lt8911_parse_dt(struct device *dev,
                           struct lt8911_data *pdata)
{
    struct device_node *np = dev->of_node;

    pdata->pwr_gpio = of_get_named_gpio(np, "power-gpio", 0);
    if (!gpio_is_valid(pdata->pwr_gpio)) {
        dev_err(dev, "No valid pwr gpio");
        return -1;
    }

    pdata->rst_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (!gpio_is_valid(pdata->rst_gpio)) {
        dev_err(dev, "No valid rst gpio");
        return -1;
    }

    return 0;
}

static int lt8911_request_io_port(struct lt8911_data *lt8911)
{
    int ret = 0;

    if (gpio_is_valid(lt8911->pwr_gpio)) {
        ret = gpio_request(lt8911->pwr_gpio, "lt8911_pwr");
        if (ret < 0) {
            dev_err(&lt8911->client->dev,
                    "Failed to request GPIO:%d, ERRNO:%d\n",
                    (s32)lt8911->pwr_gpio, ret);
            return -ENODEV;
        }

        gpio_direction_input(lt8911->pwr_gpio);
        dev_info(&lt8911->client->dev, "Success request pwr-gpio\n");
    }

    if (gpio_is_valid(lt8911->rst_gpio)) {
        ret = gpio_request(lt8911->rst_gpio, "lt8911_rst");
        if (ret < 0) {
            dev_err(&lt8911->client->dev,
                    "Failed to request GPIO:%d, ERRNO:%d\n",
                    (s32)lt8911->rst_gpio, ret);

            if (gpio_is_valid(lt8911->pwr_gpio))
                gpio_free(lt8911->pwr_gpio);

            return -ENODEV;
        }

        gpio_direction_input(lt8911->rst_gpio);
        dev_info(&lt8911->client->dev,  "Success request rst-gpio\n");
    }

    return 0;
}

static int lt8911_i2c_test(struct i2c_client *client)
{
    u8 retry = 0;
    u8 chip_id_h = 0, chip_id_l = 0;
    int ret = -EAGAIN;

    while (retry++ < 3) {
        ret = lt8911_write(client, 0xff, 0x80);
        if (ret < 0) {
            dev_err(&client->dev, "LT8911 i2c test write addr:0xff failed\n");
            continue;
        }

        chip_id_l = lt8911_read(client, 0x00);
        chip_id_h = lt8911_read(client, 0x01);
        dev_info(&client->dev, "LT8911 i2c test success chipid: 0x%x%x\n", chip_id_h, chip_id_l);

//        if (chip_id_h == 0 && chip_id_l == 0) {
//            dev_err(&client->dev, "LT8911 i2c test failed time %d\n", retry);
//            continue;
//        }

        ret = 0;
        break;
    }

    return ret;
}

void lt8911_dpcd_write(struct i2c_client *client, u32 address, u8 Data)
{
    u8 address_h = 0x0f & (address >> 16);
    u8 address_m = 0xff & (address >> 8);
    u8 address_l = 0xff & address;

    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x62, 0xbd);
    lt8911_write(client, 0x62, 0xbf);   // ECO(AUX reset)

    lt8911_write(client, 0x36, 0x00);
    lt8911_write(client, 0x30, 0x0f);   // 0x11
    lt8911_write(client, 0x33, address_l);
    lt8911_write(client, 0x34, address_m);
    lt8911_write(client, 0x35, address_h);
    lt8911_write(client, 0x37, Data);
    lt8911_write(client, 0x36, 0x20);
}

u8 lt8911_dpcd_read(struct i2c_client *client, u32 address)
{
    u8 read_cnt = 0x03;
    u8 dpcd_value = 0x00;
    u8 address_h = 0x0f & (address >> 16);
    u8 address_m = 0xff & (address >> 8);
    u8 address_l = 0xff & address;

    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x62, 0xbd);
    lt8911_write(client, 0x62, 0xbf); //ECO(AUX reset)

    lt8911_write(client, 0x36, 0x00);
    lt8911_write(client, 0x30, 0x8f); //0x91
    lt8911_write(client, 0x33, address_l);
    lt8911_write(client, 0x34, address_m);
    lt8911_write(client, 0x35, address_h);
    lt8911_write(client, 0x36, 0x20);

    mdelay(2); //The necessary

    if(lt8911_read(client, 0x39) == 0x01) {
        dpcd_value = lt8911_read(client, 0x38);
    } else {
        while((lt8911_read(client, 0x39) != 0x01) && (read_cnt > 0)) {
            lt8911_write(client, 0x36, 0x00);
            lt8911_write(client, 0x36, 0x20);
            read_cnt--;
            mdelay(2);
        }

        if(lt8911_read(client, 0x39) == 0x01) {
            dpcd_value = lt8911_read(client, 0x38);
        }
    }
    return dpcd_value;
}


void lt8911_adj_swing(struct i2c_client *client)
{
    u8 ret = 0;

    swing_req = DPCD0206H & 0x0f;   //lane 0
    lt8911_write(client, 0xff, 0x81);

    switch(swing_req) {
        case 0x00: //0dB_400mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0xa0);
            lt8911_write(client, 0x11, 0x00);
            ret = 0x00;
            break;

        case 0x01: //0dB_600mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0xd0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x00);
            ret = 0x01;
            break;

        case 0x02: //0dB_800mV
            lt8911_write(client, 0x18, 0x01);
            lt8911_write(client, 0x19, 0x20);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x00);
            ret = 0x02;
            break;

        case 0x03: //0dB_1200mV(max 1000mV)
            lt8911_write(client, 0x18, 0x01);
            lt8911_write(client, 0x19, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x00);
            ret = 0x07;
            break;

        case 0x04: //3.5dB_400mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0x98);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x28);
            ret = 0x08;
            break;

        case 0x05: //3.5dB_600mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0xf0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x38);
            ret = 0x09;
            break;

        case 0x06: //3.5dB_800mV
            lt8911_write(client, 0x18, 0x01);
            lt8911_write(client, 0x19, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x70);
            ret = 0x0a;
            break;

        case 0x07:
            break;

        case 0x08: //6dB_400mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x44);
            ret = 0x10;
            break;

        case 0x09: //6dB_800mV
            lt8911_write(client, 0x18, 0x01);
            lt8911_write(client, 0x19, 0x00);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x58);
            ret = 0x11;
            break;

        case 0x0a:
            break;

        case 0x0b:
            ret = 0x17;
            break;

        case 0x0c: //9.5dB_400mV
            lt8911_write(client, 0x18, 0x00);
            lt8911_write(client, 0x19, 0xc0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x11, 0x78);
            ret = 0x78;
            break;

        case 0x0d:
            break;

        case 0x0e:
            ret = 0x3a;
            break;

        case 0x0f:
            break;

        default:
            break;
    }

    lt8911_dpcd_write(client, 0x0103, ret);

#ifdef _2_LANE_

    ret = 0x00;

    swing_req = DPCD0206H & 0xf0;   //lane 1
    lt8911_write(client, 0xff, 0x81);

    switch(swing_req) {
        case 0x00: //0dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xa0);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x00;
            break;

        case 0x10: //0dB_600mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xd0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x01;
            break;

        case 0x20: //0dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0x20);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x02;
            break;

        case 0x30: //0dB_1200mV(max 1000mV)
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x07;
            break;

        case 0x40: //3.5dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0x98);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x28);
            ret = 0x08;
            break;

        case 0x50: //3.5dB_600mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xf0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x38);
            ret = 0x09;
            break;

        case 0x60: //3.5dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x70);
            ret = 0x0a;
            break;

        case 0x70:
            break;

        case 0x80:  //6dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x44);
            ret = 0x12;
            break;

        case 0x90:  //6dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0x00);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x58);
            ret = 0x13;
            break;

        case 0xa0:
            break;

        case 0xb0:
            ret = 0x17;
            break;

        case 0xc0:  //9.5dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xc0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x78);
            ret = 0x78;
            break;

        case 0xd0:
            break;

        case 0xe0:
            ret = 0x3a;
            break;

        case 0xf0:
            break;

        default:
            break;
    }

    lt8911_dpcd_write(client, 0x0104, ret);

#endif

#ifdef _4_LANE_
    ret = 0x00;

    swing_req = DPCD0206H & 0xf0;   //lane 1
    lt8911_write(client, 0xff, 0x81);

    switch(swing_req) {
        case 0x00: //0dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xa0);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x00;
            break;

        case 0x10: //0dB_600mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xd0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x01;
            break;

        case 0x20: //0dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0x20);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x02;
            break;

        case 0x30: //0dB_1200mV(max 1000mV)
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x00);
            ret = 0x07;
            break;

        case 0x40: //3.5dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0x98);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x28);
            ret = 0x08;
            break;

        case 0x50: //3.5dB_600mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xf0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x38);
            ret = 0x09;
            break;

        case 0x60: //3.5dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x70);
            ret = 0x0a;
            break;

        case 0x70:
            break;

        case 0x80:  //6dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x44);
            ret = 0x12;
            break;

        case 0x90:  //6dB_800mV
            lt8911_write(client, 0x1a, 0x01);
            lt8911_write(client, 0x1b, 0x00);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x58);
            ret = 0x13;
            break;

        case 0xa0:
            break;

        case 0xb0:
            ret = 0x17;
            break;

        case 0xc0:  //9.5dB_400mV
            lt8911_write(client, 0x1a, 0x00);
            lt8911_write(client, 0x1b, 0xc0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x13, 0x78);
            ret = 0x78;
            break;

        case 0xd0:
            break;

        case 0xe0:
            ret = 0x3a;
            break;

        case 0xf0:
            break;

        default:
            break;
    }

    lt8911_dpcd_write(client, 0x0104, ret);

    ret = 0x00;

    swing_req = DPCD0207H & 0x0f;   //lane 0
    lt8911_write(client, 0xff, 0x81);

    switch(swing_req) {
        case 0x00: //0dB_400mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0xa0);
            lt8911_write(client, 0x15, 0x00);
            ret = 0x00;
            break;

        case 0x01: //0dB_600mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0xd0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x00);
            ret = 0x01;
            break;

        case 0x02: //0dB_800mV
            lt8911_write(client, 0x1c, 0x01);
            lt8911_write(client, 0x1d, 0x20);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x00);
            ret = 0x02;
            break;

        case 0x03: //0dB_1200mV(max 1000mV)
            lt8911_write(client, 0x1c, 0x01);
            lt8911_write(client, 0x1d, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x00);
            ret = 0x07;
            break;

        case 0x04: //3.5dB_400mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0x98);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x28);
            ret = 0x08;
            break;

        case 0x05: //3.5dB_600mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0xf0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x38);
            ret = 0x09;
            break;

        case 0x06: //3.5dB_800mV
            lt8911_write(client, 0x1c, 0x01);
            lt8911_write(client, 0x1d, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x70);
            ret = 0x0a;
            break;

        case 0x07:
            break;

        case 0x08:  //6dB_400mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0xa0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x44);
            ret = 0x10;
            break;

        case 0x09:  //6dB_800mV
            lt8911_write(client, 0x1c, 0x01);
            lt8911_write(client, 0x1d, 0x00);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x58);
            ret = 0x11;
            break;

        case 0x0a:
            break;

        case 0x0b:
            ret = 0x17;
            break;

        case 0x0c:  //9.5dB_400mV
            lt8911_write(client, 0x1c, 0x00);
            lt8911_write(client, 0x1d, 0xc0);
            //lt8911_write(client,0x10,0x00);
            lt8911_write(client, 0x15, 0x78);
            ret = 0x78;
            break;

        case 0x0d:
            break;

        case 0x0e:
            ret = 0x3a;
            break;

        case 0x0f:
            break;

        default:
            break;
    }

    lt8911_dpcd_write(client, 0x0105, ret);

    ret = 0x00;

    swing_req = DPCD0207H & 0xf0;   //lane 1
    lt8911_write(client, 0xff, 0x81);

    switch(swing_req) {
        case 0x00: //0dB_400mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0xa0);
            lt8911_write(client, 0x17, 0x00);
            ret = 0x00;
            break;

        case 0x10: //0dB_600mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0xd0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x00);
            ret = 0x01;
            break;

        case 0x20: //0dB_800mV
            lt8911_write(client, 0x1e, 0x01);
            lt8911_write(client, 0x1f, 0x20);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x00);
            ret = 0x02;
            break;

        case 0x30: //0dB_1200mV(max 1000mV)
            lt8911_write(client, 0x1e, 0x01);
            lt8911_write(client, 0x1f, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x00);
            ret = 0x07;
            break;

        case 0x40: //3.5dB_400mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0x98);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x28);
            ret = 0x08;
            break;

        case 0x50: //3.5dB_600mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0xf0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x38);
            ret = 0x09;
            break;

        case 0x60: //3.5dB_800mV
            lt8911_write(client, 0x1e, 0x01);
            lt8911_write(client, 0x1f, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x70);
            ret = 0x0a;
            break;

        case 0x70:
            break;

        case 0x80:  //6dB_400mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0xa0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x44);
            ret = 0x12;
            break;

        case 0x90:  //6dB_800mV
            lt8911_write(client, 0x1e, 0x01);
            lt8911_write(client, 0x1f, 0x00);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x58);
            ret = 0x13;
            break;

        case 0xa0:
            break;

        case 0xb0:
            ret = 0x17;
            break;

        case 0xc0:  //9.5dB_400mV
            lt8911_write(client, 0x1e, 0x00);
            lt8911_write(client, 0x1f, 0xc0);
            //lt8911_write(client,0x12,0x00);
            lt8911_write(client, 0x17, 0x78);
            ret = 0x78;
            break;

        case 0xd0:
            break;

        case 0xe0:
            ret = 0x3a;
            break;

        case 0xf0:
            break;

        default:
            break;
    }

    lt8911_dpcd_write(client, 0x0106, ret);

#endif

}


void LT8911_AUX_Training(struct i2c_client *client)
{
    u8 swing_adj_cnt = 0x00;
    DPCD0202H  = 0x00;
#ifdef _4_LANE_
    DPCD0203H = 0x00;
#endif
    DPCD0000H  = lt8911_dpcd_read(client, 0x0000);
    DPCD0200H  = lt8911_dpcd_read(client, 0x0200);
    DPCD0201H  = lt8911_dpcd_read(client, 0x0201);
    DPCD0202H  = lt8911_dpcd_read(client, 0x0202);
    DPCD0203H  = lt8911_dpcd_read(client, 0x0203);
    DPCD0204H  = lt8911_dpcd_read(client, 0x0204);
    DPCD0205H  = lt8911_dpcd_read(client, 0x0205);
    DPCD0000H  = lt8911_dpcd_read(client, 0x0000);
    DPCD0001H  = lt8911_dpcd_read(client, 0x0001);
    DPCD0002H  = lt8911_dpcd_read(client, 0x0002);
    DPCD0003H  = lt8911_dpcd_read(client, 0x0003);
    DPCD0004H  = lt8911_dpcd_read(client, 0x0004);
    DPCD0005H  = lt8911_dpcd_read(client, 0x0005);
    DPCD0006H  = lt8911_dpcd_read(client, 0x0006);
    DPCD0007H  = lt8911_dpcd_read(client, 0x0007);
    DPCD0008H  = lt8911_dpcd_read(client, 0x0008);
    DPCD0009H  = lt8911_dpcd_read(client, 0x0009);
    DPCD000aH  = lt8911_dpcd_read(client, 0x000a);
    DPCD000bH  = lt8911_dpcd_read(client, 0x000b);

    lt8911_write(client, 0xff, 0x80); //register bank
    lt8911_write(client, 0x62, 0x3f); //Reset dp video

#ifdef _4_LANE_
    lt8911_write(client, 0x03, 0x44); //41-1lane,42-2lane,44-4lane
#endif

#ifdef _2_LANE_
    lt8911_write(client, 0x03, 0x42); //41-1lane,42-2lane,44-4lane
#endif

#ifdef _1_LANE_
    lt8911_write(client, 0x03, 0x41); //41-1lane,42-2lane,44-4lane
#endif

    lt8911_write(client, 0x65, 0xf1);
    mdelay(2);
    lt8911_write(client, 0x65, 0xf3);

    lt8911_write(client, 0x04, 0x14);

    lt8911_write(client, 0xff, 0x84); //register bank
//  lt8911_write(client,0x14,0x01);
    lt8911_write(client, 0x14, 0x81);
    lt8911_write(client, 0x14, 0x82);

    lt8911_dpcd_write(client, 0x0600, 0x01);

    if(lt8911_dpcd_read(client, 0x0600) != 0x01) {
        lt8911_dpcd_write(client, 0x0600, 0x01);
    }

    lt8911_dpcd_write(client, 0x0100, 0x0a);

#ifdef _4_LANE_
    lt8911_dpcd_write(client, 0x0101, 0x84); //4 lane
#endif

#ifdef _2_LANE_
    lt8911_dpcd_write(client, 0x0101, 0x82); //2 lane
#endif

#ifdef _1_LANE_
    lt8911_dpcd_write(client, 0x0101, 0x81); //1 lane
#endif

    lt8911_dpcd_write(client, 0x010a, 0x00);
//  lt8911_dpcd_write(client, 0x0107,0x00);
//  lt8911_dpcd_write(client, 0x0108,0x01);

    if(lt8911_dpcd_read(client, 0x0100) != 0x0a) {
        lt8911_dpcd_write(client, 0x0100, 0x0a);
    }

#ifdef _4_LANE_
    if(lt8911_dpcd_read(client, 0x0101) != 0x84) { //1080P 2 Lane
        lt8911_dpcd_write(client, 0x0101, 0x84);
    }
#endif

#ifdef _2_LANE_
    if(lt8911_dpcd_read(client, 0x0101) != 0x82) { //2 Lane
        lt8911_dpcd_write(client, 0x0101, 0x82);
    }
#endif

#ifdef _1_LANE_
    if(lt8911_dpcd_read(client, 0x0101) != 0x81) { //1 Lane
        lt8911_dpcd_write(client, 0x0101, 0x81);
    }
#endif

    if(lt8911_dpcd_read(client, 0x010a) != 0x00) {
        lt8911_dpcd_write(client, 0x010a, 0x00);
    }

//  lt8911_dpcd_write(client, 0x0102,0x00);
    lt8911_dpcd_write(client, 0x0102, 0x01); //sent TPS1
    lt8911_dpcd_write(client, 0x0103, 0x00);

#ifdef _2_LANE_
    lt8911_dpcd_write(client, 0x0104, 0x00);
#endif

#ifdef _4_LANE_
    lt8911_dpcd_write(client, 0x0104, 0x00);
    lt8911_dpcd_write(client, 0x0105, 0x00);
    lt8911_dpcd_write(client, 0x0106, 0x00);
#endif
    if(lt8911_dpcd_read(client, 0x0102) != 0x01) {
        lt8911_dpcd_write(client, 0x0102, 0x01);
    }

    mdelay(16);
    DPCD0204H  = lt8911_dpcd_read(client, 0x0204);
    DPCD0202H  = lt8911_dpcd_read(client, 0x0202);
#ifdef _4_LANE_
    DPCD0203H = lt8911_dpcd_read(client, 0x0203);
#endif

    swing_adj_cnt = 0x05;

#ifdef _4_LANE_
    DPCD0203H = DPCD0203H & 0x11; // 1080P 0x11 ; 1366 0x01
    while(((DPCD0203H & 0x11) != 0x11) && (swing_adj_cnt > 0))  // 1080P 0x11 ; 1366 0x01
#endif

#ifdef _2_LANE_
    DPCD0202H = DPCD0202H & 0x11; // 2 Lane 0x11 ; 1 Lane 0x01
    while(((DPCD0202H & 0x11) != 0x11) && (swing_adj_cnt > 0)) // 1080P 0x11 ; 1366 0x01
#endif

#ifdef _1_LANE_
    DPCD0202H = DPCD0202H & 0x01; // 2 Lane 0x11 ; 1 Lane 0x01
    while(((DPCD0202H & 0x01) != 0x01) && (swing_adj_cnt > 0)) // 1080P 0x11 ; 1366 0x01
#endif
    {
        DPCD0206H = lt8911_dpcd_read(client, 0x0206);
#ifdef _4_LANE_
        DPCD0207H = lt8911_dpcd_read(client, 0x0207);
#endif
        lt8911_adj_swing(client);
        swing_adj_cnt--;
        mdelay(1);
        DPCD0202H = lt8911_dpcd_read(client, 0x0202);
#ifdef _4_LANE_
        DPCD0203H = lt8911_dpcd_read(client, 0x0203);
#endif

#ifdef _4_LANE_
        DPCD0203H = DPCD0203H & 0x11; // 4 lane 0x11 ; 1 lane 0x01
#endif

#ifdef _2_LANE_
        DPCD0202H = DPCD0202H & 0x11; // 2 Lane 0x11 ; 1 Lane 0x01
#endif

#ifdef _1_LANE_
        DPCD0202H = DPCD0202H & 0x01; // 2 Lane 0x11 ; 1 Lane 0x01
#endif
    }

//  lt8911_write(client,0xff,0x82); //for debug
//  lt8911_write(client,0x1b,DPCD0202H);

#ifdef _4_LANE_
    if(DPCD0203H == 0x11) // 4 lane 0x11 ; 1 lane 0x01
#endif

#ifdef _2_LANE_
    if(DPCD0202H == 0x11) // 2 Lane 0x11 ; 1 Lane 0x01
#endif

#ifdef _1_LANE_
    if(DPCD0202H == 0x01) // 2 Lane 0x11 ; 1 Lane 0x01
#endif
    {
        lt8911_write(client, 0xff, 0x80); //register bank
        lt8911_write(client, 0x04, 0x18);

        lt8911_write(client, 0xff, 0x84); //register bank
        //  lt8911_write(client,0x14,0x04);
        lt8911_write(client, 0x14, 0x84);
        lt8911_write(client, 0x14, 0x88); //0x88

        lt8911_dpcd_write(client, 0x0102, 0x02); // sent TPS2
        if(lt8911_dpcd_read(client, 0x0102) != 0x02) {
            lt8911_dpcd_write(client, 0x0102, 0x02);
        }

        mdelay(16);
        DPCD0204H  = lt8911_dpcd_read(client, 0x0204);
        DPCD0202H  = lt8911_dpcd_read(client, 0x0202);
#ifdef _4_LANE_
        DPCD0203H = lt8911_dpcd_read(client, 0x0203);
#endif

        swing_adj_cnt = 0x05;

#ifdef _4_LANE_
        while(((DPCD0203H & 0x77) != 0x77) && (swing_adj_cnt > 0)) // 4 lane 0x77 ; 1 lane 0x07
#endif

#ifdef _2_LANE_
        while(((DPCD0202H & 0x77) != 0x77) && (swing_adj_cnt > 0)) // 2 Lane 0x77 ; 1 Lane 0x07
#endif

#ifdef _1_LANE_
        while(((DPCD0202H & 0x07) != 0x07) && (swing_adj_cnt > 0)) // 2 Lane 0x77 ; 1 Lane 0x07
#endif
        {
            DPCD0206H = lt8911_dpcd_read(client, 0x0206);
#ifdef _4_LANE_
            DPCD0207H = lt8911_dpcd_read(client, 0x0207);
#endif
            lt8911_write(client, 0xff, 0x84); //register bank
            lt8911_write(client, 0x14, 0x08);
            lt8911_write(client, 0x14, 0x88);
            lt8911_adj_swing(client);
            swing_adj_cnt--;
            mdelay(1);
            DPCD0202H  = lt8911_dpcd_read(client, 0x0202);
#ifdef _4_LANE_
            DPCD0203H = lt8911_dpcd_read(client, 0x0203);
#endif
            DPCD0204H  = lt8911_dpcd_read(client, 0x0204);
        }
    }

    lt8911_write(client, 0xff, 0x84); //register bank
//  lt8911_write(client,0x14,0x04);
    lt8911_write(client, 0x14, 0x84);
    lt8911_write(client, 0x14, 0x88);

    lt8911_write(client, 0xff, 0x80); //register bank
    lt8911_write(client, 0x04, 0x18);

    lt8911_dpcd_write(client, 0x0102, 0x02);

//  lt8911_write(client,0xff,0x82);//register bank
//  lt8911_write(client,0x1c,DPCD0202H);

    mdelay(2);

    lt8911_dpcd_write(client, 0x0102, 0x00);

    lt8911_write(client, 0xff, 0x80); //register bank
    lt8911_write(client, 0x04, 0x10);

    lt8911_write(client, 0xff, 0x84); //register bank
    lt8911_write(client, 0x14, 0x80);
    lt8911_write(client, 0x14, 0xc0);

    lt8911_write(client, 0xff, 0x88); //register bank

    if(lt8911_read(client, 0x24) != 0xc0) {
        lt8911_write(client, 0xff, 0x80); //register bank
        lt8911_write(client, 0x62, 0x3f);
        //  lt8911_write(client, 0x62, 0xbf);
    }

//  lt8911_write(client, 0xff, 0x80); //register bank
//  lt8911_write(client, 0x62, 0xbf);

    lt8911_write(client, 0xff, 0x80); //register bank
    lt8911_write(client, 0x65, 0xf1);
    mdelay(2);
    lt8911_write(client, 0x65, 0xf3);

    lt8911_dpcd_write(client, 0x0102, 0x00); // sent data

    if(lt8911_dpcd_read(client, 0x0102) != 0x00) {
        lt8911_dpcd_write(client, 0x0102, 0x00);
    }

    if(lt8911_dpcd_read(client, 0x0600) != 0x01) {
        lt8911_dpcd_write(client, 0x0600, 0x01);
    }

    if(lt8911_dpcd_read(client, 0x010a) != 0x00) {
        lt8911_dpcd_write(client, 0x010a, 0x00);
    }

    DPCD0202H = lt8911_dpcd_read(client, 0x0202);
#ifdef _4_LANE_
    DPCD0203H = lt8911_dpcd_read(client, 0x0203);
#endif
}


void lt8911_crc_polling(struct i2c_client *client)
{
    u8 crc_ready = 0x00;

    //LVDS CRC
    lt8911_write(client, 0xff, 0x82);           //
    lt8911_write(client, 0x10, 0x81);           //CRC clear

    crc_ready = lt8911_read(client, 0x04);      //CRC_rdy

    if(crc_ready == 0x02) {                     // no lvds clk, or PLL power down
        lt8911_write(client, 0xff, 0x80);
        lt8911_write(client, 0x62, 0x3f);       //edp video_RST

        lt8911_write(client, 0xff, 0x80);       //
        lt8911_write(client, 0x63, 0x7f);       // LVDS Rx Pll rst
        lt8911_write(client, 0x63, 0xff);       //
    } else {
        lt8911_write(client, 0x10, 0x01);       //
        lt8911_write(client, 0x10, 0x41);       //CRC strat
        mdelay(100);                            //more than 40ms

        crc_ready = lt8911_read(client, 0x04);  //CRC_rdy

        if(crc_ready == 0x00) {                 // no lvds clk, or PLL power down
            lt8911_write(client, 0xff, 0x80);
            lt8911_write(client, 0x62, 0x3f);   //edp video_RST

            lt8911_write(client, 0xff, 0x80);   //
            lt8911_write(client, 0x63, 0x7f);   // LVDS Rx Pll rst
            lt8911_write(client, 0x63, 0xff);   //
        }
        lt8911_write(client, 0xff, 0x80);
        lt8911_write(client, 0x62, 0xbf);       //edp video_RST
    }
}


void lt8911_setup(struct i2c_client *client)
{
    lt8911_write(client, 0xff, 0x81);   //register bank
    lt8911_write(client, 0x00, 0x04);

#ifdef _LANE_RATE_16G_TX_ //1.62G
    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x7a, _SSC_);
    lt8911_write(client, 0x71, 0x20);   //0x36-->20
    lt8911_write(client, 0x72, 0x99);   //0x00-->99
    lt8911_write(client, 0x73, 0x19);   //0x00-->19
    lt8911_write(client, 0x63, 0x7f);
    lt8911_write(client, 0x63, 0xff);

    lt8911_write(client, 0xff, 0x81);
    lt8911_write(client, 0x0e, 0x37);
    lt8911_write(client, 0x01, 0x18);
    lt8911_write(client, 0x02, 0x42);
    lt8911_write(client, 0x04, 0x00);
    lt8911_write(client, 0x04, 0x01);

    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x61, 0x7f);
    lt8911_write(client, 0x61, 0xff);   //txpll reset

    lt8911_write(client, 0xff, 0x81);
    lt8911_write(client, 0x05, 0x1b);   // for 1.62G

    //txpll_digtal
    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x74, 0x41);   //prd[7:0]
    lt8911_write(client, 0x75, 0x03);   //prd[13:8]
    lt8911_write(client, 0x76, 0x0a);   //delta1[7:0]
    lt8911_write(client, 0x78, 0x0a);   //delta[7:0]
#endif

#ifdef _LANE_RATE_27G_TX_ //2.7G 
    lt8911_write(client, 0xff, 0x80);   //register bank
    lt8911_write(client, 0x7a, 0x07);
    lt8911_write(client, 0x71, 0x36);
    lt8911_write(client, 0x72, 0x00);
    lt8911_write(client, 0x73, 0x00);

    lt8911_write(client, 0x63, 0x7f);
    lt8911_write(client, 0x63, 0xff);

    lt8911_write(client, 0xff, 0x81);   //register bank
    lt8911_write(client, 0x0e, 0x37);
    lt8911_write(client, 0x01, 0x18);
    lt8911_write(client, 0x02, 0x42);
    lt8911_write(client, 0x04, 0x00);
    lt8911_write(client, 0x04, 0x01);

    lt8911_write(client, 0xff, 0x80);   //register bank
    lt8911_write(client, 0x61, 0x7f);
    lt8911_write(client, 0x61, 0xff);

    lt8911_write(client, 0xff, 0x81);   //register bank
    lt8911_write(client, 0x05, 0x13);

    lt8911_write(client, 0xff, 0x80);   //register bank
    lt8911_write(client, 0x74, 0x41);
    lt8911_write(client, 0x75, 0x03);
    lt8911_write(client, 0x76, 0x0a);
    lt8911_write(client, 0x78, 0x0a);
#endif

    lt8911_write(client, 0xff, 0x81);   //register bank
    lt8911_write(client, 0x0e, 0x37);

#ifdef _1_LANE_
    // 1 Lane eDP Output
    lt8911_write(client, 0x22, 0x11);   // 关闭 LANE1 / LANE2 / LANE3 SWING 的电流开关;0x10->0x11: enable pre-emphase driver
    lt8911_write(client, 0x23, 0x3e);   // 关闭 LANE1 / LANE2 / LANE3 pre-emphase 的电流开关
    lt8911_write(client, 0x25, 0x08);

    lt8911_write(client, 0x18, SWING_LEVEL_0_H);
    lt8911_write(client, 0x19, SWING_LEVEL_0_L);
#endif

#ifdef _2_LANE_
    // 2 Lane eDP Output
    lt8911_write(client, 0x22, 0x33);   // 关闭 LANE2 / LANE3 SWING的电流开关;0x30->0x33: enable pre-emphase driver
    lt8911_write(client, 0x23, 0x3c);   // 关闭 LANE2 / LANE3 pre-emphase的电流开关
    lt8911_write(client, 0x25, 0x08);

    lt8911_write(client, 0x18, SWING_LEVEL_0_H);
    lt8911_write(client, 0x19, SWING_LEVEL_0_L);

    lt8911_write(client, 0x1a, SWING_LEVEL_0_H);
    lt8911_write(client, 0x1b, SWING_LEVEL_0_L);
#endif

#ifdef _4_LANE_
    lt8911_write(client, 0x22, 0xff);
    lt8911_write(client, 0x23, 0x30);
    lt8911_write(client, 0x25, 0x08);

    lt8911_write(client, 0x18, SWING_LEVEL_0_H);
    lt8911_write(client, 0x19, SWING_LEVEL_0_L);

    lt8911_write(client, 0x1a, SWING_LEVEL_1_H);
    lt8911_write(client, 0x1b, SWING_LEVEL_1_L);

    lt8911_write(client, 0x1c, SWING_LEVEL_2_H);
    lt8911_write(client, 0x1d, SWING_LEVEL_2_L);

    lt8911_write(client, 0x1e, SWING_LEVEL_3_H);
    lt8911_write(client, 0x1f, SWING_LEVEL_3_L);
#endif

#ifdef _TEST_PATTERN_
#ifdef _1080P_EDP_PANEL_
    /*1080P 148.5MHz*/
    lt8911_write(client, 0xff, 0x90);
    lt8911_write(client, 0x4a, 0x33);   //0x99
    lt8911_write(client, 0x4b, 0x33);   //0x99
    lt8911_write(client, 0x4c, 0xd3);   //0x69
    lt8911_write(client, 0x4d, 0x10);   //0x10:software DDS; 0x00:Rx DDS
#endif

#ifdef _1366x768_EDP_PANEL_
    /*1366x768 72MHz*/
    lt8911_write(client, 0xff, 0x90);
    lt8911_write(client, 0x4a, 0x66);   //0x99
    lt8911_write(client, 0x4b, 0x66);   //0x99
    lt8911_write(client, 0x4c, 0x66);   //0x69
    lt8911_write(client, 0x4d, 0x10);   //0x10:software DDS; 0x00:Rx DDS
#endif

#ifdef _1600x900_EDP_PANEL_
    /*1600x900  89.8 MHz*/
    lt8911_write(client, 0xff, 0x90);
    lt8911_write(client, 0x4a, 0x30);   //0x99
    lt8911_write(client, 0x4b, 0xbf);   //0x99
    lt8911_write(client, 0x4c, 0x7f);   //0x69
    lt8911_write(client, 0x4d, 0x10);   //0x10:software DDS; 0x00:Rx DDS
#endif

#ifdef _1280x800_eDP_Panel_
    /*1600x900  89.8 MHz*/
    lt8911_write(client, 0xff, 0x90);
    lt8911_write(client, 0x4a, 0x4f);
    lt8911_write(client, 0x4b, 0xfa);
    lt8911_write(client, 0x4c, 0x64);
    lt8911_write(client, 0x4d, 0x10);   //0x10:software DDS; 0x00:Rx DDS
#endif

#ifdef _1920x1200_EDP_PANEL_
    /*1920x1200  148 MHz*/
    lt8911_write(client, 0xff, 0x90);
    lt8911_write(client, 0x4a, 0x27);
    lt8911_write(client, 0x4b, 0x7d);
    lt8911_write(client, 0x4c, 0xd2);
    lt8911_write(client, 0x4d, 0x10);   //0x10:software DDS; 0x00:Rx DDS
#endif
#else
    lt8911_write(client, 0xff, 0x90);   //register bank
    lt8911_write(client, 0x4d, 0x00);   //Close DDS
#endif

    lt8911_write(client, 0xff, 0x81);   //register bank

#ifdef _TEST_PATTERN_
    lt8911_write(client, 0x09, 0x01);   //reference clk=lvds clk
    lt8911_write(client, 0x0b, 0x0b);
    lt8911_write(client, 0x08, 0x13);
#else
    lt8911_write(client, 0x09, 0x07);
    lt8911_write(client, 0x0b, 0x05);
#ifdef _2PORT_LVDS_INPUT_
    lt8911_write(client, 0x08, 0x2b);   // 2 port LVDS input
#else
    lt8911_write(client, 0x08, 0x26);   // 1 port LVDS input
#endif
#endif

    lt8911_write(client, 0xff, 0x80);   //register bank
    lt8911_write(client, 0x63, 0x7f);
    lt8911_write(client, 0x63, 0xff);

    lt8911_write(client, 0xff, 0x81);   //register bank
    lt8911_write(client, 0x27, 0x80);
    lt8911_write(client, 0x28, 0xa4);
    lt8911_write(client, 0x29, 0x52);
    lt8911_write(client, 0x2a, 0x07);
    lt8911_write(client, 0x2b, 0x02);
    lt8911_write(client, 0x2c, 0x02);
    lt8911_write(client, 0x2d, 0x02);
    lt8911_write(client, 0x2e, 0xaa);
    lt8911_write(client, 0x2f, 0x02);
    lt8911_write(client, 0x30, 0xaa);
    lt8911_write(client, 0x31, 0x03);
    lt8911_write(client, 0x32, 0x4a);
    lt8911_write(client, 0x33, 0x20);
    lt8911_write(client, 0x34, 0x00);
    lt8911_write(client, 0x35, 0x80);
    lt8911_write(client, 0x36, 0xa4);
    lt8911_write(client, 0x37, 0x52);
    lt8911_write(client, 0x38, 0x03);
    lt8911_write(client, 0x39, 0x36);
    lt8911_write(client, 0x3a, 0x00);

    lt8911_write(client, 0xff, 0x80);   //register bank
    lt8911_write(client, 0x13, 0x01);
    lt8911_write(client, 0xff, 0x82);   //register bank
    lt8911_write(client, 0x10, 0x01);

#ifdef _2PORT_LVDS_INPUT_
    lt8911_write(client, 0x11, 0x00);   //LVDS Out Sync Pol Inverse
#else
    lt8911_write(client, 0x11, 0x08);   //LVDS Out Sync Pol Inverse
#endif

#ifdef _8_BIT_COLOR_DEPTH_
#ifdef _DE_MODE_
    lt8911_write(client, 0x12, 0x00);
#else
    lt8911_write(client, 0x12, 0x36);
#endif
#endif

#ifdef _6_BIT_COLOR_DEPTH_
#ifdef _DE_MODE_
    lt8911_write(client, 0x12, 0x80);
#else
    lt8911_write(client, 0x12, 0xB6);
#endif
#endif

#ifdef _2PORT_LVDS_INPUT_
    lt8911_write(client, 0x13, 0x65);

    //在参考设计中，LVDS的DB3N、DB3P是接到LT8911的42、43脚的。
    lt8911_write(client, 0x14, 0x87); // for 8911 Reference Design

    //在LT8911 demo板中，LVDS的DB3N、DB3P是接到LT8911的44、45脚的。
    //lt8911_write(client,0x14,0x97);// for 8911 Demo Board

    lt8911_write(client, 0x15, 0x50);
    lt8911_write(client, 0x16, 0x76);
    lt8911_write(client, 0x17, 0x48);
#endif

    lt8911_write(client, 0x18, 0x03);

    lt8911_write(client, 0x34, 0x00);
    lt8911_write(client, 0x35, 0x20);
    lt8911_write(client, 0x36, 0x22);
    lt8911_write(client, 0x37, 0x22);
    lt8911_write(client, 0x38, 0x22);
    lt8911_write(client, 0x39, 0x22);

#ifdef _DE_MODE_
#ifdef _2PORT_LVDS_INPUT_
    lt8911_write(client, 0x3a, (u8)((LVDS_timing[H_tol] - LVDS_timing[H_act] - LVDS_timing[H_bp] - LVDS_timing[H_sync]) / 2)); // RG_H_FP[7:0]
    lt8911_write(client, 0x3b, (u8)(LVDS_timing[H_sync] / 2));     //RG_H_SYNC_WID[7:0]
    lt8911_write(client, 0x3d, (u8)((LVDS_timing[H_tol] / 2) % 256));   //RG_H_TOTAL[7:0]
    lt8911_write(client, 0x3e, 0xa0 + (u8)((LVDS_timing[H_tol] / 2) / 256));  //[7]=0 sync mode;[7]=0 DE mode;
    lt8911_write(client, 0x3f, ((u8)(LVDS_timing[V_sync])) * 16 + (u8)(LVDS_timing[V_tol] - LVDS_timing[V_act] - LVDS_timing[V_bp] - LVDS_timing[V_sync]));
#else
    // 1 port LVDS input
    lt8911_write(client, 0x3a, (u8)(LVDS_timing[H_tol] - LVDS_timing[H_act] - LVDS_timing[H_bp] - LVDS_timing[H_sync])); //RG_H_FP[7:0]
    lt8911_write(client, 0x3b, (u8)(LVDS_timing[H_sync])); //RG_H_SYNC_WID[7:0]
    lt8911_write(client, 0x3d, (u8)(LVDS_timing[H_tol] % 256));  //RG_H_TOTAL[7:0]
    lt8911_write(client, 0x3e, 0xa0 + (u8)(LVDS_timing[H_tol] / 256));  //[7]=0 sync mode;[7]=0 DE mode;
    lt8911_write(client, 0x3f, ((u8)(LVDS_timing[V_sync])) * 16 + (u8)(LVDS_timing[V_tol] - LVDS_timing[V_act] - LVDS_timing[V_bp] - LVDS_timing[V_sync]));
#endif
#else
    // Sync mode
    lt8911_write(client, 0x3e, 0x20);
#endif

    lt8911_write(client, 0xff, 0x8c);
    lt8911_write(client, 0x00, 0x00);
    lt8911_write(client, 0x01, 0x80);
    lt8911_write(client, 0x02, 0x00);

    lt8911_write(client, 0xff, 0x88);                                                   //register bank
    lt8911_write(client, 0x00, 0x6a);
    lt8911_write(client, 0x04, 0xff);

    lt8911_write(client, 0x05, (u8)(LVDS_timing[H_tol] / 256));                         //RG_HTOTAL[15:0]
    lt8911_write(client, 0x06, (u8)(LVDS_timing[H_tol] % 256));                         //RG_HTOTAL[7:0]
    lt8911_write(client, 0x07, (u8)((LVDS_timing[H_bp] + LVDS_timing[H_sync]) / 256));  //RG_HSTART [15:8]
    lt8911_write(client, 0x08, (u8)((LVDS_timing[H_bp] + LVDS_timing[H_sync]) % 256));  //RG_HSTART[7:0]

#ifdef _TEST_PATTERN_
    lt8911_write(client, 0x09, (u8)(LVDS_timing[H_sync] / 256));                         //[7]RG_HSPOL;[6:0]RG_HSYNC_WIDTH[14:8]
    lt8911_write(client, 0x0a, (u8)(LVDS_timing[H_sync] % 256));                         //RG_HSYNC_WIDTH[7:0]=60
#else
    lt8911_write(client, 0x09, 0x00);                                                    //[7]RG_HSPOL;[6:0]RG_HSYNC_WIDTH[14:8]
    lt8911_write(client, 0x0a, 0x00);                                                    //RG_HSYNC_WIDTH[7:0]=60
#endif
    lt8911_write(client, 0x0b, (u8)(LVDS_timing[H_act] / 256));                          //RG_HWIDTH[15:8]
    lt8911_write(client, 0x0c, (u8)(LVDS_timing[H_act] % 256));                          //RG_HWIDTH[7:0]
    lt8911_write(client, 0x0d, (u8)(LVDS_timing[V_tol] / 256));                          //RG_VTOTAL [15:8]
    lt8911_write(client, 0x0e, (u8)(LVDS_timing[V_tol] % 256));                          //RG_VTOTAL[7:0]

    lt8911_write(client, 0x0f, 0x00);                                                    //RG_TOP_VTOTAL[15:8] //fiexd
    lt8911_write(client, 0x10, 0x00);                                                    //RG_TOP_VTOTAL[7:0] //fixed

    lt8911_write(client, 0x11, (u8)((LVDS_timing[V_bp] + LVDS_timing[V_sync]) / 256));   //RG_VSTART[15:8]
    lt8911_write(client, 0x12, (u8)((LVDS_timing[V_bp] + LVDS_timing[V_sync]) % 256));   //RG_VSTART[7:0]

#ifdef _TEST_PATTERN_
    lt8911_write(client, 0x13, (u8)(LVDS_timing[V_sync] / 256));                          //RG_VSPOL;RG_VSYNC_WIDTH[14:8]
    lt8911_write(client, 0x14, (u8)(LVDS_timing[V_sync] % 256));                          //RG_VSYNC_WIDTH[7:0]
#else
    lt8911_write(client, 0x13, 0x00);  //RG_VSPOL;RG_VSYNC_WIDTH[14:8]
    lt8911_write(client, 0x14, 0x00);                                                     //RG_VSYNC_WIDTH[7:0]
#endif
    lt8911_write(client, 0x15, (u8)(LVDS_timing[V_act] / 256));                           //RG_VHEIGTH[15:8]
    lt8911_write(client, 0x16, (u8)(LVDS_timing[V_act] % 256));                           //RG_VHEIGTH[7:0]

#ifdef _6_BIT_COLOR_DEPTH_
    lt8911_write(client, 0x17, 0x00);                                                     // LVDS Color Depth:   6 bit: 0x00 ;
    lt8911_write(client, 0x18, 0x00);                                                     // LVDS Color Depth:   6 bit: 0x00 ;
#endif

#ifdef _8_BIT_COLOR_DEPTH_
    lt8911_write(client, 0x17, 0x08);                                                     // LVDS Color Depth:   8 bit: 0x08
    lt8911_write(client, 0x18, 0x20);                                                     // LVDS Color Depth:   8 bit: 0x20
#endif

    lt8911_write(client, 0x19, 0x00);
    lt8911_write(client, 0x1a, 0x80);
    lt8911_write(client, 0x1e, 0x30);
    lt8911_write(client, 0x21, 0x00);

#ifdef _TEST_PATTERN_
    lt8911_write(client, 0x2c, 0xdf);
#else
    lt8911_write(client, 0x2c, 0xd0);
#endif

    lt8911_write(client, 0x2d, 0x00);
    lt8911_write(client, 0x4b, 0xfe);

    lt8911_write(client, 0x2e, (u8)((LVDS_timing[V_bp] + LVDS_timing[V_sync]) % 256));    //RG_GCM_DE_TOP[6:0]
    lt8911_write(client, 0x2f, (u8)((LVDS_timing[H_bp] + LVDS_timing[H_sync]) / 256));    //RG_GCM_DE_DLY[11:8]
    lt8911_write(client, 0x30, (u8)((LVDS_timing[H_bp] + LVDS_timing[H_sync]) % 256));    //RG_GCM_DE_DLY[7:0]
    lt8911_write(client, 0x31, (u8)(LVDS_timing[H_act] / 256));                           //RG_GCM_DE_CNT[11:8]
    lt8911_write(client, 0x32, (u8)(LVDS_timing[H_act] % 256));                           //RG_GCM_DE_CNT[7:0]
    lt8911_write(client, 0x33, (u8)(LVDS_timing[V_act] / 256));                           //RG_GCM_DE_LIN[10:8]
    lt8911_write(client, 0x34, (u8)(LVDS_timing[V_act] % 256));                           //RG_GCM_DE_LIN[7:0]
    lt8911_write(client, 0x35, (u8)(LVDS_timing[H_tol] / 256));                           //RG_GCM_HTOTAL[11:8]
    lt8911_write(client, 0x36, (u8)(LVDS_timing[H_tol] % 256));                           //RG_GCM_HTOTAL[7:0]

#ifdef _TEST_PATTERN_
    lt8911_write(client, 0x37, 0x18 + (u8)(LVDS_timing[V_tol] / 256));
#else
    lt8911_write(client, 0x37, 0x18 + (u8)(LVDS_timing[V_tol] / 256));
#endif

    lt8911_write(client, 0x38, (u8)(LVDS_timing[V_tol] % 256));                           //RG_GCM_VTOTAL[7:0]
    lt8911_write(client, 0x39, 0x00);   // reseve
    lt8911_write(client, 0x3a, ((u8)(LVDS_timing[V_sync] % 256)) * 4 + (u8)(LVDS_timing[H_sync] / 256));
    lt8911_write(client, 0x3b, (u8)(LVDS_timing[H_sync] % 256));                          //RG_GCM_HWIDTH[7:0]

    /*Training*/
    LT8911_AUX_Training(client);

#ifdef _4_LANE_
    if(DPCD0203H != 0x77)
#endif

#ifdef _2_LANE_
    if(DPCD0202H != 0x77)
#endif

#ifdef _1_LANE_
    if(DPCD0202H != 0x07)
#endif
    LT8911_AUX_Training(client);

    lt8911_write(client, 0xff, 0x80); //register bank
    lt8911_write(client, 0x62, 0x3f);

    lt8911_write(client, 0x63, 0x7f);
    lt8911_write(client, 0x63, 0xff);

    mdelay(100);

#ifndef _TEST_PATTERN_
    lt8911_write(client, 0xff, 0x88);
    lt8911_write(client, 0x37, 0x08+(u8)(LVDS_timing[V_tol]/256));
    mdelay(100);
#endif

    lt8911_write(client, 0xff, 0x80);
    lt8911_write(client, 0x62, 0xbf);

    lt8911_write(client, 0xff, 0x00); //register bank
}

static int lt8911_probe(struct i2c_client * client, const struct i2c_device_id * id)
{
    int ret = -1;
    struct lt8911_data *lt8911;

    /* do NOT remove these logs */
    dev_info(&client->dev, "LT8911 Driver Version: %s\n", LT8911_DRIVER_VERSION);
    dev_info(&client->dev, "LT8911 I2C Address: 0x%02x\n", client->addr);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "Failed check I2C functionality");
        return -ENODEV;
    }

    lt8911 = devm_kzalloc(&client->dev, sizeof(*lt8911), GFP_KERNEL);
    if (lt8911 == NULL) {
        dev_err(&client->dev, "Failed alloc lt8911 memory");
        return -ENOMEM;
    }

    if (client->dev.of_node) {
        ret = lt8911_parse_dt(&client->dev, lt8911);
        if (ret < 0) {
            dev_err(&client->dev, "Failed parse dlt8911\n");
            goto exit_free_client_data;
        }
    }

    lt8911->client = client;
    i2c_set_clientdata(client, lt8911);

    ret = lt8911_request_io_port(lt8911);
    if (ret < 0) {
        dev_err(&client->dev, "Failed request IO port\n");
        goto exit_free_client_data;
    }

//    lt8911_reset_guitar(client);

    ret = lt8911_i2c_test(client);
    if (ret < 0) {
        dev_err(&client->dev, "Failed communicate with IC use I2C\n");
        goto exit_free_io_port;
    }

    lt8911_setup(client);
    dev_info(&client->dev, "LT8911 setup finish.\n");

    return 0;

exit_free_io_port:
    if (gpio_is_valid(lt8911->rst_gpio))
        gpio_free(lt8911->rst_gpio);
    if (gpio_is_valid(lt8911->pwr_gpio))
        gpio_free(lt8911->pwr_gpio);
exit_free_client_data:
    devm_kfree(&client->dev, lt8911);
    i2c_set_clientdata(client, NULL);

    return ret;
}

static int lt8911_remove(struct i2c_client * client)
{
    struct lt8911_data *lt8911 = i2c_get_clientdata(client);

    if (gpio_is_valid(lt8911->rst_gpio))
        gpio_free(lt8911->rst_gpio);

    if (gpio_is_valid(lt8911->pwr_gpio))
        gpio_free(lt8911->pwr_gpio);

    dev_info(&client->dev, "goodix lt8911 driver removed");
    i2c_set_clientdata(client, NULL);

    devm_kfree(&client->dev, lt8911);

    return 0;
}

static const struct of_device_id lt8911_match_table[] = {
    {.compatible = "lontium,lt8911",},
    { },
};

static const struct i2c_device_id lt8911_device_id[] = {
    { LT8911_I2C_NAME, 0 },
    { }
};

static struct i2c_driver lt8911_driver = {
    .probe      = lt8911_probe,
    .remove     = lt8911_remove,
    .id_table   = lt8911_device_id,
    .driver = {
        .name     = LT8911_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = lt8911_match_table,
    },
};

static int __init lt8911_init(void)
{
    s32 ret;

    pr_info("Lontium LT8911 driver installing....\n");
    ret = i2c_add_driver(&lt8911_driver);

    return ret;
}

static void __exit lt8911_exit(void)
{
    pr_info("Lontium LT8911 driver exited\n");
    i2c_del_driver(&lt8911_driver);
}

module_init(lt8911_init);
module_exit(lt8911_exit);

MODULE_DESCRIPTION("Lontium LT8911 Driver");
MODULE_LICENSE("GPL V2");

