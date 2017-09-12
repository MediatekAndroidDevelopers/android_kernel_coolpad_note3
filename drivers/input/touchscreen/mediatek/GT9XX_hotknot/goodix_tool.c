/* drivers/input/touchscreen/mediatek/gt9xx_mtk/goodix_tool.c
 *
 * Copyright  (C)  2010 - 2016 Goodix., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
 * General Public License for more details.
 *
 * Version: V2.6
 */

#include "tpd.h"
#include "include/tpd_gt9xx_common.h"
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>	/*proc */

#pragma pack(1)
struct st_cmd_head {
	u8 wr;			/* write read flag��0:R1:W2:PID 3: */
	u8 flag;		/* 0:no need flag/int 1: need flag2:need int */
	u8 flag_addr[2];	/* flag address */
	u8 flag_val;		/* flag val */
	u8 flag_relation;	/* flag_val:flag 0:not equal 1:equal 2:> 3:< */
	u16 circle;		/* polling cycle */
	u8 times;		/* plling times */
	u8 retry;		/* I2C retry times */
	u16 delay;		/* delay before read or after write */
	u16 data_len;		/* data length */
	u8 addr_len;		/* address length */
	u8 addr[2];		/* address */
	u8 res[3];		/* reserved */
	u8 *data;		/* data pointer */
};
#pragma pack()
struct st_cmd_head cmd_head, cmd_head2;

#define DATA_LENGTH_UINT	512
#define CMD_HEAD_LENGTH (sizeof(struct st_cmd_head) - sizeof(u8 *))

static struct i2c_client *gt_client;

static s32 (*tool_i2c_read)(u8*, u16);
static s32 (*tool_i2c_write)(u8*, u16);

static s32 DATA_LENGTH = 0;
static s8 IC_TYPE[16] = "GT9XX";

#ifdef CONFIG_HOTKNOT_BLOCK_RW
DECLARE_WAIT_QUEUE_HEAD(bp_waiter);
u8 got_hotknot_state = 0;
u8 got_hotknot_extra_state = 0;
u8 wait_hotknot_state = 0;
u8 force_wake_flag = 0;
#endif

static s32 tool_i2c_read_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;

	ret = gtp_i2c_read(gt_client, buf, len + GTP_ADDR_LENGTH);
	return ret;
}

static s32 tool_i2c_write_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;

	ret = gtp_i2c_write(gt_client, buf, len);
	return ret;
}

static s32 tool_i2c_read_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = { 0x0f, 0xff };
	u8 end[2] = { 0x80, 0x00 };

	tool_i2c_write_no_extra(pre, 2);
	ret = tool_i2c_read_no_extra(buf, len);
	tool_i2c_write_no_extra(end, 2);

	return ret;
}

static s32 tool_i2c_write_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = { 0x0f, 0xff };
	u8 end[2] = { 0x80, 0x00 };

	tool_i2c_write_no_extra(pre, 2);
	ret = tool_i2c_write_no_extra(buf, len);
	tool_i2c_write_no_extra(end, 2);

	return ret;
}

static void register_i2c_func(void)
{
/* if (!strncmp(IC_TYPE, "GT818", 5) || !strncmp(IC_TYPE, "GT816", 5) */
/* || !strncmp(IC_TYPE, "GT811", 5) || !strncmp(IC_TYPE, "GT818F", 6) */
/* || !strncmp(IC_TYPE, "GT827", 5) || !strncmp(IC_TYPE,"GT828", 5) */
/* || !strncmp(IC_TYPE, "GT813", 5)) */
	if (strncmp(IC_TYPE, "GT8110", 6) && strncmp(IC_TYPE, "GT8105", 6)
	    && strncmp(IC_TYPE, "GT801", 5) && strncmp(IC_TYPE, "GT800", 5)
	    && strncmp(IC_TYPE, "GT801PLUS", 9) && strncmp(IC_TYPE, "GT811", 5)
	    && strncmp(IC_TYPE, "GTxxx", 5) && strncmp(IC_TYPE, "GT9XX", 5)) {
		tool_i2c_read = tool_i2c_read_with_extra;
		tool_i2c_write = tool_i2c_write_with_extra;
		GTP_DEBUG("I2C function: with pre and end cmd!");
	} else {
		tool_i2c_read = tool_i2c_read_no_extra;
		tool_i2c_write = tool_i2c_write_no_extra;
		GTP_INFO("I2C function: without pre and end cmd!");
	}
}

static void unregister_i2c_func(void)
{
	tool_i2c_read = NULL;
	tool_i2c_write = NULL;
	GTP_INFO("I2C function: unregister i2c transfer function!");
}

s32 init_wr_node(struct i2c_client *client)
{
	s32 i;

	gt_client = i2c_client_point;

	memset(&cmd_head, 0, sizeof(cmd_head));
	cmd_head.data = NULL;

	i = 5;

	while ((!cmd_head.data) && i) {
		cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);

		if (NULL != cmd_head.data)
			break;

		i--;
	}

	if (i) {
		DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
		GTP_INFO("Applied memory size:%d.", DATA_LENGTH);
	} else {
		GTP_ERROR("Apply for memory failed.");
		return FAIL;
	}

	cmd_head.addr_len = 2;
	cmd_head.retry = 5;

	memset(&cmd_head2, 0, sizeof(cmd_head2));
	cmd_head2.data = NULL;

	i = 5;

	while ((!cmd_head2.data) && i) {
		cmd_head2.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);

		if (NULL != cmd_head2.data)
			break;

		i--;
	}

	if (i) {
		DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
		GTP_INFO("Applied memory size:%d.", DATA_LENGTH);
	} else {
		GTP_ERROR("Apply for memory failed.");
		return FAIL;
	}

	cmd_head2.addr_len = 2;
	cmd_head2.retry = 5;

	register_i2c_func();

	return SUCCESS;
}

void uninit_wr_node(void)
{
	kfree(cmd_head.data);
	cmd_head.data = NULL;
	kfree(cmd_head2.data);
	cmd_head2.data = NULL;
	unregister_i2c_func();
}
