/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* N2DM.h
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * mpu300 head file for MT65xx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef N2DM_H
#define N2DM_H
	 
#include <linux/ioctl.h>
	 
#define N2DM_I2C_SLAVE_ADDR		0xD0
#define N2DM_FIXED_DEVID			0x33

/* N2DM Register Map  (Please refer to N2DM Specifications) */
#define N2DM_REG_STATUS_REG_AUX 0x07
#define N2DM_REG_OUT_TEMP_L 0x0c
#define N2DM_REG_OUT_TEMP_H 0x0d
#define N2DM_REG_INT_COUNTER_REG 0x0e
#define N2DM_REG_WHO_AM_I 0x0f

#define N2DM_REG_TEMP_CFG_REG 0x1f
#define N2DM_REG_CTRL1_REG 0x20
#define N2DM_REG_CTRL2_REG 0x21
#define N2DM_REG_CTRL3_REG 0x22
#define N2DM_REG_CTRL4_REG 0x23
#define N2DM_REG_CTRL5_REG 0x24
#define N2DM_REG_CTRL6_REG 0x25

#define N2DM_REG_REFERENCE_DATACAPTURE 0x26
#define N2DM_REG_STATUS_REG 0x27
#define N2DM_REG_OUT_X_L 0x28
#define N2DM_REG_OUT_X_H 0x29
#define N2DM_REG_OUT_Y_L 0x2a
#define N2DM_REG_OUT_Y_H 0x2b
#define N2DM_REG_OUT_Z_L 0x2c
#define N2DM_REG_OUT_Z_H 0x2d

#define N2DM_REG_FIFO_CTRL_REG 0x2e
#define N2DM_REG_FIFO_SRC_REG 0x2f

#define N2DM_REG_INT1_CFG 0x30
#define N2DM_REG_INT1_SRC 0x31
#define N2DM_REG_INT1_THS 0x32
#define N2DM_REG_INT1_DURATION 0x33
#define N2DM_REG_INT2_CFG 0x34
#define N2DM_REG_INT2_SRC 0x35
#define N2DM_REG_INT2_THS 0x36
#define N2DM_REG_INT2_DURATION 0x37

#define N2DM_REG_CLICK_CFG 0x38
#define N2DM_REG_CLICK_SRC 0x39
#define N2DM_REG_CLICK_THS 0x3a

#define N2DM_REG_TIME_LIMIT 0x3b
#define N2DM_REG_TIME_LATENCY 0x3c
#define N2DM_REG_TIME_WINDOW 0x3D
#define N2DM_REG_ACT_THS 0x3E
#define N2DM_REG_ACT_DUR 0x3F


/*N2DM Register Bit definitions*/
#define N2DM_ACC_RANGE_MASK 0x30
#define N2DM_ACC_RANGE_2g		    0x00
#define N2DM_ACC_RANGE_4g		    0x10
#define N2DM_ACC_RANGE_8g		    0x20
#define N2DM_ACC_RANGE_16g		    0x30

#define N2DM_ACC_ODR_MASK	0xF0
#define N2DM_ACC_ODR_POWER_DOWN		0x00
#define N2DM_ACC_ODR_1HZ		    0x10
#define N2DM_ACC_ODR_10HZ		    0x20
#define N2DM_ACC_ODR_25HZ		    0x30
#define N2DM_ACC_ODR_50HZ		    0x40
#define N2DM_ACC_ODR_100HZ		    0x50
#define N2DM_ACC_ODR_200HZ		    0x60
#define N2DM_ACC_ODR_400HZ		    0x70
#define N2DM_ACC_ODR_1620HZ_LP		    			0x80
#define N2DM_ACC_ODR_NOR_1344HZ_LP_5376HZ		    0x90

#define N2DM_WORK_MODE_MASK 0x08
#define N2DM_WORK_MODE_HR_SET 0x08
#define N2DM_WORK_MODE_LP_SET 0x08
#define N2DM_WORK_MODE_NM_SET 0x00

#define N2DM_WORK_NORMAL_MODE 0
#define N2DM_WORK_LP_MODE 1
#define N2DM_WORK_HR_MODE 2


#define I2C_AUTO_INCREMENT 0x80

#define N2DM_ACC_SENSITIVITY_2G_NORMAL_MODE		4	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_2G_HR_MODE			1	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_2G_LP_MODE			16	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_4G_NORMAL_MODE		8	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_4G_HR_MODE			2	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_4G_LP_MODE			32	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_8G_NORMAL_MODE		16	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_8G_HR_MODE			4	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_8G_LP_MODE			64	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_16G_NORMAL_MODE		48	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_16G_HR_MODE			12	/*mg/LSB*/
#define N2DM_ACC_SENSITIVITY_16G_LP_MODE			192	/*mg/LSB*/



#define N2DM_ACC_ENABLE_AXIS_MASK 0x07
#define N2DM_ACC_ENABLE_AXIS_X 0x01
#define N2DM_ACC_ENABLE_AXIS_Y 0x02
#define N2DM_ACC_ENABLE_AXIS_Z 0x04


#define N2DM_SUCCESS		       0
#define N2DM_ERR_I2C		      -1
#define N2DM_ERR_STATUS			  -3
#define N2DM_ERR_SETUP_FAILURE	  -4
#define N2DM_ERR_GETGSENSORDATA  -5
#define N2DM_ERR_IDENTIFICATION	  -6

#define N2DM_BUFSIZE 60


// begin baibo@yulong for acc_calibration 20150906
#define N2DM_IOCTL_MAGIC      'd'
#define ACC_CALIBRATE   _IOR(N2DM_IOCTL_MAGIC, 6, short)

struct acc_offset{
    signed short key;   //calibrate status
    signed short x;     //x offset
    signed short y;     //y offset
    signed short z;     //z offset
};
// end baibo@yulong for acc_calibration 20150906

/*------------------------------------------------------------------*/

// 1 rad = 180/PI degree, L3G4200D_OUT_MAGNIFY = 131,
// 180*131/PI = 7506
#define DEGREE_TO_RAD	180*1000000/PI//7506  // fenggy mask
//#define DEGREE_TO_RAD 819	 
#endif //N2DM_H

