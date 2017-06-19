/********************************************************************************/
/*																			                                       	*/
/* Copyright (c) 2000-2012  YULONG Company             　　　　　　　       	  */
/*         宇龙计算机通信科技（深圳）有限公司  版权所有 2000-2012               */
/*																				                                      */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the           			    */
/* subject matter of this material.  All manufacturing, reproduction, use,      */
/* and sales rights pertaining to this subject matter are governed by the     	*/
/* license agreement.  The recipient of this sofpanicare implicitly accepts    */
/* the terms of the license.                                                    */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得     			    */
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                             */
/*																				                                      */
/********************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:          yl_panic.c
**  Author:            樊立
**  Version :          1.00
**  Date:              2012-11-20
**  Description:       参数分区驱动
**
**  History:
**  <author>      <time>      <version >      <desc>
**   樊  立      2012-11-20     1.00           创建
**
**************************************************************************/
/* drivers/mmc/card/yl_panic.c
 *
 * Copyright (C) 2012 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * This sofpanicare is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Sofpanicare Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>

#include <linux/genhd.h>
#include <linux/reboot.h>
#include <linux/yl_panic.h>
#include <linux/mutex.h>

/* I define transition phase, to make system/app groups' life easy.
 * During transition phase, system/app can mix use yl_panic and yl_panic1.
 * System/app can keep on using old V0 specification for YL panic partition,
 * and still access yl_panic, disregard V1 specification.
 * As Mr. Zhang urge them to conform V1 specification,
 * I hope system/app can transist to V1 as soon as possible.
 * When most people switched to V1 specification,
 * we will close transistion phase, and yl_panic will dispear,
 * only left those V0 people struggle along.
 * So, KEEP your eyes on.
 * */

// #define PANIC_TRANSITION  //deleted by fanli ,it's just fit to 9960,2012.11.20
//#define pr_fmt(fmt)	"YLLOG: YL-panic: " fmt
#if 1   //modified by fanli for 7295 ,2012.11.20
#define YL_PANIC_BLOCK_NUM	  12      //added by fanli, just fit MTK6589 Platform, 2013.01.17
#define YL_PANIC_NAME	"panic"   //modified by fanli for 7295 ,2012.11.20
#define YL_MISC_NAME	"MISC"     //modified by fanli for 7295 ,2012.11.20
#else
/* on NVIDIA, partition name is 3-characters */
#define YL_PANIC_NAME	"PAR"
#define YL_MISC_NAME	"MSC"
#endif

#define PANIC_TAG  "YL-panic: "
//#define PANIC_DEBUG  0//close it when upload CC,fanli,2012.11.27
#ifdef PANIC_DEBUG
#define PANIC_DBG(fmt, ...) \
	printk(PANIC_TAG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define PANIC_DBG(fmt, ...) do{ \
}while(0)
#endif
const char *yl_panic_map[] = {
	[YL_DEVICE]		    = "DEVICE",
	[YL_CONFIGURATION]	= "CONFIGURATION",
	[YL_PRODUCTLINE]	= "PRODUCTLINE",
	[YL_DYNAMIC]		= "DYNAMIC",
	[YL_GUARD]		    = "GUARD",
	[YL_CMDLINE]		= "CMDLINE",
	[YL_TOUCHSCREEN0]	= "TOUCHSCREEN0",
	[YL_TOUCHSCREEN1]	= "TOUCHSCREEN1",
	[YL_TOUCHSCREEN2]	= "TOUCHSCREEN2",
	[YL_TOUCHSCREEN3]	= "TOUCHSCREEN3",
	[YL_RESERVE0]		= "RESERVE0",
	[YL_RESERVE1]		= "RESERVE1",
	[YL_PROJECT0]		= "PROJECT0",
	[YL_PROJECT1]		= "PROJECT1",
	[YL_PROJECT2]		= "PROJECT2",
	[YL_PROJECT3]		= "PROJECT3",
	[YL_FETCH_PASSWD]	= "FETCH_PASSWD",
	[YL_FCT_DIAG]		= "FCT_DIAG",
	[YL_RCP]	        = "RCP",
	[YL_RETURNZERO]		= "RETURNZERO",
};

enum preload_state {
	PRELOAD_EMPTY = 0,
	PRELOAD_CLEAR,
	PRELOAD_DIRTY
};

static enum preload_state preload[YL_PANIC_COUNT];
static char yl_panic[YL_PANIC_COUNT][ONE_BLOCK_SIZE];

static struct miscdevice yl_panic_dev1;

/* resources should be protect by yl_panic_lock:
 * preload, yl_panic, kernel_buf,
 * */
static DEFINE_MUTEX(yl_panic_lock);

struct yl_panic_data {
	struct mmc_card	*card;
	struct hd_struct *part;
};
static struct yl_panic_data yl_panic_priv;
static struct yl_panic_data yl_misc_priv;

/*
 * Fill in the mmc_request structure given a set of transfer paniceters.
 */
static void yl_panic_prepare_mrq(struct mmc_card *card,
		struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
		unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Wait for the card to finish the busy state
 */
static int yl_panic_wait_busy(struct mmc_card *card)
{
	int ret, busy;
	struct mmc_command cmd;

	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && !(cmd.resp[0] & R1_READY_FOR_DATA)) {
			busy = 1;
			PANIC_DBG("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
		}
	} while (!(cmd.resp[0] & R1_READY_FOR_DATA));

	return ret;
}

/*
 * Checks that a normal transfer didn't have any errors
 */
static int yl_panic_check_res(struct mmc_request *mrq)
{
	int ret;

	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	if (mrq->cmd->error)
		ret = mrq->cmd->error;
	else if (mrq->data->error)
		ret = mrq->data->error;
	else if (mrq->stop && mrq->stop->error)
		ret = mrq->stop->error;
	else if (mrq->data->bytes_xfered !=
			mrq->data->blocks * mrq->data->blksz)
		ret = -EIO;
	else
		ret = mrq->data->bytes_xfered;

	if (ret < 0)
		PANIC_DBG("%s return ret=%d\n", __func__, ret);
	return ret;
}

static int yl_panic_transfer(struct yl_panic_data *priv,
		sector_t offset, char *buf, int write)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_command stop;
	struct mmc_data data;
	struct scatterlist sg;
	struct mmc_card *card;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&stop, 0, sizeof(struct mmc_command));

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	sg_init_one(&sg, buf, ONE_BLOCK_SIZE);

	card = priv->card;

	mmc_claim_host(card->host);
	yl_panic_prepare_mrq(card, &mrq, &sg, 1,
			priv->part->start_sect + offset,
			1, ONE_BLOCK_SIZE, write);
	mmc_wait_for_req(card->host, &mrq);
	yl_panic_wait_busy(card);
	mmc_release_host(card->host);

	return yl_panic_check_res(&mrq);
}

static int yl_panic_get_pageoffset(const char *buf)
{
	int i;

	for (i = 0; i < YL_PANIC_COUNT; i++) {
		if (!strncmp(yl_panic_map[i], buf, TAG_LENGTH)) {
			PANIC_DBG("find tag name is [%s]\n", buf);
			return i;
		}
	}
	return -ERANGE;
}

/* caller should only provide buffer of size 512 bytes exactly,
 * */
static ssize_t yl_panic_read_locked(char *buf)
{
	int block;
	ssize_t len = ONE_BLOCK_SIZE;

	block = yl_panic_get_pageoffset(buf);
	if (block < 0) {
		PANIC_DBG("Can't find tag %s, line %d!\n", buf, __LINE__);
		return -ERANGE;
	}
        PANIC_DBG("block=%d at %s func\n", block, __func__);
	if (preload[block] == PRELOAD_EMPTY) {
		len = yl_panic_transfer(&yl_panic_priv, block,
				yl_panic[block], false);
		if (len == ONE_BLOCK_SIZE)
			preload[block] = PRELOAD_CLEAR;
	}
	if (len == ONE_BLOCK_SIZE)
		memcpy(buf, yl_panic[block], ONE_BLOCK_SIZE);
	else
		PANIC_DBG("read block %s failed, ret %ld\n", buf, len);
	return len;
}

/* caller should only provide buffer of size 512 bytes exactly,
 * as only read-modify-write is accept
 * */
static ssize_t yl_panic_write_locked(char *buf)
{
	int block;
	ssize_t len;
	int retry = 0;

	block = yl_panic_get_pageoffset(buf);
	if(block < 0) {
		PANIC_DBG("Can't find tag %s, line %d!\n", buf, __LINE__);
		return -ERANGE;
	}

	while (1) {
		len = yl_panic_transfer(&yl_panic_priv, block, buf, true);
		if (len == ONE_BLOCK_SIZE) {
			memcpy(yl_panic[block], buf, ONE_BLOCK_SIZE);
			preload[block] = PRELOAD_CLEAR;
			break;
		} else {
			preload[block] = PRELOAD_DIRTY;
			retry ++;
			if (retry < 3)
				PANIC_DBG("write block %s failed, retry\n", buf);
			else {
				PANIC_DBG("write block %s failed 3 times, stop\n", buf);
				break;
			}
		}
	}
	/* FIXME if write transfer failed, what data is in EMMC really?
	 * Is it same as pre-loaded, or as written buf, or not consistent?
	 * And what should we deal with pre-loaded data?
	 * */
        PANIC_DBG("block=%d at %s func\n", block, __func__);
	return len;
}

ssize_t yl_panic_kernel_read(char *buf, ssize_t count)
{
	ssize_t len;

	if (count != ONE_BLOCK_SIZE)
		return -EINVAL;

	mutex_lock(&yl_panic_lock);
	len = yl_panic_read_locked(buf);
	mutex_unlock(&yl_panic_lock);
        PANIC_DBG("block=%ld,count=%ld at %s\n", buf, count, __func__);
	return len;
}
EXPORT_SYMBOL_GPL(yl_panic_kernel_read);

ssize_t yl_panic_kernel_write(char *buf, ssize_t count)
{
	ssize_t len;

	if (count != ONE_BLOCK_SIZE)
		return -EINVAL;

	mutex_lock(&yl_panic_lock);
	len = yl_panic_write_locked(buf);
	mutex_unlock(&yl_panic_lock);
        PANIC_DBG("block=%d,count=%d at %s func\n", buf, count, __func__);
	return len;
}
EXPORT_SYMBOL_GPL(yl_panic_kernel_write);

static char kernel_buf[ONE_BLOCK_SIZE];
static ssize_t yl_panic_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t len;
	size_t count_temp;

	if (count <= TAG_LENGTH)
		PANIC_DBG("%s read count %d\n", __func__, count);

	mutex_lock(&yl_panic_lock);
	count_temp = copy_from_user(kernel_buf, buf,
			count < TAG_LENGTH ? count : TAG_LENGTH);
	if (count_temp != 0)
		PANIC_DBG("%s copy from left %d\n", __func__, count_temp);

	len = yl_panic_read_locked(kernel_buf);
	if (len > 0) {
		if (len > count)
			len = count;
		count_temp = copy_to_user(buf + TAG_LENGTH, kernel_buf + TAG_LENGTH, len > TAG_LENGTH ? len - TAG_LENGTH : 0);
		if (count_temp) {
			PANIC_DBG("%s copy to left %d\n", __func__, count_temp);
			len -= count_temp;
		}
	}
	mutex_unlock(&yl_panic_lock);
	return len;
}

static ssize_t yl_panic_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t len;

	if (count <= TAG_LENGTH)
		PANIC_DBG("%s write count %d\n", __func__, count);

	/* We only support reading a maximum of a flash block */
	if (count > ONE_BLOCK_SIZE)
		count = ONE_BLOCK_SIZE;

	mutex_lock(&yl_panic_lock);
	len = yl_panic_read_locked(kernel_buf);
	if (len > 0) {
		ssize_t res;
		res = copy_from_user(kernel_buf, buf, count);
		if (res != 0)
			PANIC_DBG("%s copy left %d\n", __func__, res);
		len = count - res;

		res = yl_panic_write_locked(kernel_buf);
		if (res <= 0)
			len = res;
	}
	mutex_unlock(&yl_panic_lock);
	return len;
}


static char *preload_str(enum preload_state state)
{
	if (state == PRELOAD_EMPTY)
		return "empty";
	if (state == PRELOAD_CLEAR)
		return "clear";
	if (state == PRELOAD_DIRTY)
		return "dirty";
	return "unknown";
}

ssize_t show_preload_panic(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	int len = 0;
	mutex_lock(&yl_panic_lock);
	for (i = 0; i < YL_PANIC_COUNT; i++)
		len += sprintf(buf + len, "%16s\t%s\n",
				yl_panic_map[i],
				preload_str(preload[i]));
	mutex_unlock(&yl_panic_lock);
	return len;
}
DEVICE_ATTR(preload_panic, S_IRUSR | S_IRGRP, show_preload_panic, NULL);


static const struct file_operations yl_panic_fops = {
	.owner		= THIS_MODULE,
	.read		= yl_panic_read,
	.write		= yl_panic_write,
};

static struct miscdevice yl_panic_dev1 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "yl_panic1",
	.fops = &yl_panic_fops,
};

static char recovery_buf[ONE_BLOCK_SIZE];
static int yl_reboot(struct notifier_block *nb, unsigned long event, void *unused)
{
	if (event == SYS_RESTART && unused && !strcmp(unused, "recovery")) {
		int ret;

		ret = yl_panic_transfer(&yl_misc_priv, 0, recovery_buf, true);
		if (!ret)
			PANIC_DBG("write recover flag FAILED!\n");
	}
	PANIC_DBG("system off event=%ld\n", event);
	return NOTIFY_DONE;
}

static struct notifier_block yl_reboot_fg_notifier = {
	.notifier_call = yl_reboot,
};

int yl_panic_init(struct mmc_card *card)
{
	unsigned int res;

	if (yl_panic_priv.card) {
		PANIC_DBG("re-init denied, old card %p, new card %p!\n",
				yl_panic_priv.card, card);
		return -EEXIST;
	}

	strcpy(recovery_buf, "boot-recovery");
	/* add ourselves to the reboot_notifier_list.
	 * We support adb reboot-recovery,
	 * system can't catch this command,
	 * so it's kernel duty to write boot-recovery flag.
	 * As we have a common reboot mechanism using scratch register,
	 * which also support recovery,
	 * why still is this flag needed?
	 * Enn, maybe we dare of power cut-off during reboot.
	 * */

	 /*----Don't register reboot notifier in MTK platform,fanli ,20121205----*/

	//res = register_reboot_notifier(&yl_reboot_fg_notifier);
	//if (res != 0)
	//PANIC_DBG("can't register reboot notifier\n");

      /*----------------- End ,fanli ,20121205--------------*/

	yl_panic_priv.card = card;
	yl_misc_priv.card = card;

	res = misc_register(&yl_panic_dev1);
	/* If panic v1 register fail, it doesn't matter */
	if (res) {
		PANIC_DBG("regist yl_panic1 failed %d\n", res);
		goto out;
	}

	device_create_file(yl_panic_dev1.this_device, &dev_attr_preload_panic);
	return 0;

out:
	//unregister_reboot_notifier(&yl_reboot_fg_notifier); //Don't register reboot notifier in MTK platform,fanli ,20121205
	PANIC_DBG("YLLOG:%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

void notify_ylpanic(struct hd_struct *part)
{
	/* long long ago, it is
	 * char *pname = part->partition_name;
	 * */
	char *pname = part->info->volname;

/*----modified by fanli use block num to transfer yl-panic partition physical address ,2012.12.01-----*/
	if (part->partno== YL_PANIC_BLOCK_NUM || (!strcmp(pname, YL_PANIC_NAME)))
		yl_panic_priv.part = part;
	else if (!strcmp(pname, YL_MISC_NAME))
		yl_misc_priv.part = part;
	else
		return;
/*----modified by fanli use block num to transfer yl-panic partition physical address ,2012.12.01-----*/

	PANIC_DBG("find %s, partno=%d start_sect=0x%08lx nr_sects=0x%08lx!\n",
			pname,
			part->partno,
			part->start_sect,
			part->nr_sects);
}
