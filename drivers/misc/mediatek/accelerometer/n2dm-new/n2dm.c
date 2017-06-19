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

/* ST N2DM Accelerometer and Gyroscope sensor driver on MTK platform
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "cust_acc.h"
#include "accel.h"
#include "n2dm.h"
#include <linux/sensors.h>/*add for chip info by qinxinjun@yulong.com in 20160504*/
//#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define N2DM_NEW_ARCH // comment baibo@yulong for old arch 20150906


//#define GPIO_N2DM_EINT_PIN GPIO_ALS_EINT_PIN
//#define GPIO_N2DM_EINT_PIN_M_EINT GPIO_ALS_EINT_PIN_M_EINT
//#define CUST_EINT_N2DM_NUM CUST_EINT_ALS_NUM


/*---------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_N2DM_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define N2DM_AXIS_X          0
#define N2DM_AXIS_Y          1
#define N2DM_AXIS_Z          2
#define N2DM_ACC_AXES_NUM        3

#define N2DM_ACC_DATA_LEN        6   
 
#define N2DM_ACC_DEV_NAME        "N2DM"

// begin baibo@yulong for device info 20150906
#define N2DM_DEVICE_INFO
#define YULONG_ACC_CALIBRATION
// end baibo@yulong for device info 20150906

// begin baibo@yulong for acc_calibration 20150906
#ifdef YULONG_ACC_CALIBRATION
//#include <linux/wakelock.h>
static int acc_data[3];
#define NAND_FLASH_WR_RD_SIZE 10
extern int yl_params_kernel_write(char *buf, size_t count);
extern int yl_params_kernel_read(char *buf, size_t count);
extern int read_acc_ps_cal_data_from_flash(unsigned int offset, char* output, int counts);
extern int write_acc_ps_cal_data_to_flash(unsigned int offset, char* input, int counts);
static int yulong_accel_Calibration(struct i2c_client *client, char *buf, int bufsize,int count,unsigned long cal_arg);
static int yulong_acc_ReadCalibration(struct i2c_client *client);
#endif

//static int nv_data[3]; //virtual yl memory for debug
//#define DEBUG_CALIBRATION
// end baibo@yulong for acc_calibration 20150906

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id n2dm_i2c_id[] = {{N2DM_ACC_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_n2dm={ I2C_BOARD_INFO(N2DM_ACC_DEV_NAME, (0x10>>1))};


/*----------------------------------------------------------------------------*/
static int n2dm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int n2dm_i2c_remove(struct i2c_client *client);
//static int n2dm_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int n2dm_init_client(struct i2c_client *client, bool enable);
static int N2DM_ReadAccRawData(struct i2c_client *client, s16 data[N2DM_ACC_AXES_NUM]);
static int n2dm_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int n2dm_acc_resume(struct i2c_client *client);
static int n2dm_SetSampleRate(struct i2c_client *client, u8 sample_rate);


/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][N2DM_ACC_AXES_NUM];
    int sum[N2DM_ACC_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct n2dm_i2c_data {
    struct i2c_client *client;
	struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t 				layout;
    /*misc*/
    //struct data_resolution *reso;
    struct work_struct	eint_work;				
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[N2DM_ACC_AXES_NUM+1];

    /*data*/
	s8                      offset[N2DM_ACC_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[N2DM_ACC_AXES_NUM+1];
	
	int 					sensitivity;
	int 					full_scale;

#if defined(CONFIG_N2DM_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
//#if defined(CONFIG_HAS_EARLYSUSPEND)
//    struct early_suspend    early_drv;
//#endif     
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
        {.compatible = "mediatek,GSENSOR_1"},
        {},
};
#endif

static struct i2c_driver n2dm_i2c_driver = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = N2DM_ACC_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = accel_of_match,
#endif
    },
	.probe      		= n2dm_i2c_probe,
	.remove    			= n2dm_i2c_remove,
//	.detect				= n2dm_i2c_detect,
//#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = n2dm_acc_suspend,
    .resume             = n2dm_acc_resume,
//#endif
	.id_table = n2dm_i2c_id,
};

// begin baibo@yulong for old arch 20150906
#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
static int n2dm_local_init(void);
static int n2dm_local_uninit(void);
static int n2dm_init_flag = -1;
#endif

typedef struct {
	int x;
	int y;
	int z;
} SENSOR_DATA;

#if defined(N2DM_NEW_ARCH)
extern struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;


static struct acc_init_info  n2dm_init_info = {
	.name   = N2DM_ACC_DEV_NAME,
	.init   = n2dm_local_init,
	.uninit = n2dm_local_uninit,
};
#endif

#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER) 
static struct sensor_init_info n2dm_init_info = {
		.name = N2DM_ACC_DEV_NAME,
		.init = n2dm_local_init,
		.uninit = n2dm_local_uninit,
};
#endif
// end baibo@yulong for old arch 20150906
/*----------------------------------------------------------------------------*/
static struct i2c_client *n2dm_i2c_client = NULL;

#if (!defined(N2DM_NEW_ARCH) && !defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
static struct platform_driver n2dm_driver;
#endif

static struct n2dm_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static bool enable_status = false;
//static bool pedo_enable_status = false;


/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[accel_n2dm] " // modify baibo@yulong for debug 20150906

#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static void n2dm_dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x10;
  u8 regdata=0;
  for(i=0; i<25 ; i++)
  {
    //dump all
    hwmsen_read_byte(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;	
  }
}

static void N2DM_power(struct acc_hw *hw, unsigned int on) 
{
#if 0
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "N2DM"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "N2DM"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;
#endif	
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int n2dm_write_rel_calibration(struct n2dm_i2c_data *obj, int dat[N2DM_ACC_AXES_NUM])
{
    obj->cali_sw[N2DM_AXIS_X] = obj->cvt.sign[N2DM_AXIS_X]*dat[obj->cvt.map[N2DM_AXIS_X]];
    obj->cali_sw[N2DM_AXIS_Y] = obj->cvt.sign[N2DM_AXIS_Y]*dat[obj->cvt.map[N2DM_AXIS_Y]];
    obj->cali_sw[N2DM_AXIS_Z] = obj->cvt.sign[N2DM_AXIS_Z]*dat[obj->cvt.map[N2DM_AXIS_Z]];
#if DEBUG		
		if(atomic_read(&obj->trace) & ADX_TRC_CALI)
		{
			GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n", 
				obj->cvt.sign[N2DM_AXIS_X],obj->cvt.sign[N2DM_AXIS_Y],obj->cvt.sign[N2DM_AXIS_Z],
				dat[N2DM_AXIS_X], dat[N2DM_AXIS_Y], dat[N2DM_AXIS_Z],
				obj->cvt.map[N2DM_AXIS_X],obj->cvt.map[N2DM_AXIS_Y],obj->cvt.map[N2DM_AXIS_Z]);
			GSE_LOG("write acc calibration data  (%5d, %5d, %5d)\n", 
				obj->cali_sw[N2DM_AXIS_X],obj->cali_sw[N2DM_AXIS_Y],obj->cali_sw[N2DM_AXIS_Z]);
		}
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int n2dm_ResetCalibration(struct i2c_client *client)
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int n2dm_ReadCalibration(struct i2c_client *client, int dat[N2DM_ACC_AXES_NUM])
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->cali_sw[N2DM_AXIS_X];
    dat[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->cali_sw[N2DM_AXIS_Y];
    dat[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->cali_sw[N2DM_AXIS_Z];

#if DEBUG		
		if(atomic_read(&obj->trace) & ADX_TRC_CALI)
		{
			GSE_LOG("Read acc calibration data  (%5d, %5d, %5d)\n", 
				dat[N2DM_AXIS_X],dat[N2DM_AXIS_Y],dat[N2DM_AXIS_Z]);
		}
#endif
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int n2dm_WriteCalibration(struct i2c_client *client, int dat[N2DM_ACC_AXES_NUM])
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[N2DM_ACC_AXES_NUM];


	GSE_FUN();
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        		
		cali[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->cali_sw[N2DM_AXIS_X];
		cali[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->cali_sw[N2DM_AXIS_Y];
		cali[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->cali_sw[N2DM_AXIS_Z]; 
		cali[N2DM_AXIS_X] += dat[N2DM_AXIS_X];
		cali[N2DM_AXIS_Y] += dat[N2DM_AXIS_Y];
		cali[N2DM_AXIS_Z] += dat[N2DM_AXIS_Z];
#if DEBUG		
		if(atomic_read(&obj->trace) & ADX_TRC_CALI)
		{
			GSE_LOG("write acc calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n", 
				dat[N2DM_AXIS_X], dat[N2DM_AXIS_Y], dat[N2DM_AXIS_Z],
				cali[N2DM_AXIS_X],cali[N2DM_AXIS_Y],cali[N2DM_AXIS_Z]);
		}
#endif
		return n2dm_write_rel_calibration(obj, cali);
	} 

	return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = N2DM_FIXED_DEVID;    

	res = hwmsen_read_byte(client,N2DM_REG_WHO_AM_I,databuf);
    GSE_LOG(" N2DM  id %x!\n",databuf[0]);
	if(databuf[0]!=N2DM_FIXED_DEVID)
	{
		return N2DM_ERR_IDENTIFICATION;
	}

	if (res < 0)
	{
		return N2DM_ERR_I2C;
	}
	
	return N2DM_SUCCESS;
}

static int n2dm_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;

	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return N2DM_SUCCESS;
	}

	if(hwmsen_read_byte(client, N2DM_REG_CTRL1_REG, databuf))
	{
		GSE_ERR("read n2dm power ctl register err!\n");
		return N2DM_ERR_I2C;
	}
	GSE_LOG("N2DM_REG_CTRL1_REG:databuf[0] =  %x!\n", databuf[0]);


	if(true == enable)
	{
		databuf[0] &= ~N2DM_ACC_ODR_MASK;//clear n2dm ODR bits
		databuf[0] |= N2DM_ACC_ODR_100HZ; //default set 100HZ for n2dm
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = N2DM_REG_CTRL1_REG;    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("n2dm set power mode: ODR 100hz failed!\n");
		return N2DM_ERR_I2C;
	}	
	else
	{
		GSE_LOG("set n2dm ODR 100HZ ok %d!\n", enable);
	}	

	sensor_power = enable;
	
	return N2DM_SUCCESS;    
}

static int n2dm_Update_Sensitivity(struct i2c_client *client)
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[6];
	bool low_power = false;
	bool high_res = false;
	memset(databuf, 0, sizeof(u8)*6);
	
//	if(NULL == sensitivity)
//	{
//		return -1;
//	}
	if(NULL == client)
	{
		//*buf = 0;
		return -2;
	}
	
	if(hwmsen_read_byte(client, N2DM_REG_CTRL1_REG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return N2DM_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
		if(databuf[0]&0x08)
		{
			low_power = true;
		}	
	}
	
	if(hwmsen_read_byte(client, N2DM_REG_CTRL4_REG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return N2DM_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);		
		if(databuf[0]&0x08)
		{
			high_res = true;
		}
	}
	
	if(low_power)
	{
		switch(obj->full_scale)
		{
			case N2DM_ACC_RANGE_2g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_LP_MODE;
				break;
			case N2DM_ACC_RANGE_4g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_4G_LP_MODE;
				break;
			case N2DM_ACC_RANGE_8g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_8G_LP_MODE;
				break;
			case N2DM_ACC_RANGE_16g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_16G_LP_MODE;
				break;
			default:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_LP_MODE;
				break;				
		}
	}
	else if(high_res)
	{
		switch(obj->full_scale)
		{
			case N2DM_ACC_RANGE_2g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_HR_MODE;
				break;
			case N2DM_ACC_RANGE_4g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_4G_HR_MODE;
				break;
			case N2DM_ACC_RANGE_8g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_8G_HR_MODE;
				break;
			case N2DM_ACC_RANGE_16g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_16G_HR_MODE;
				break;
			default:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_HR_MODE;
				break;				
		}
	}
	else
	{
		switch(obj->full_scale)
		{
			case N2DM_ACC_RANGE_2g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_NORMAL_MODE;
				break;
			case N2DM_ACC_RANGE_4g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_4G_NORMAL_MODE;
				break;
			case N2DM_ACC_RANGE_8g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_8G_NORMAL_MODE;
				break;
			case N2DM_ACC_RANGE_16g:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_16G_NORMAL_MODE;
				break;
			default:
				obj->sensitivity = N2DM_ACC_SENSITIVITY_2G_NORMAL_MODE;
				break;				
		}
	}
	
	return 0;	
}

static int n2dm_SetWorkMode(struct i2c_client *client, u8 work_mode)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
	GSE_FUN();     
	
	if(N2DM_WORK_HR_MODE == work_mode)
	{
		if(hwmsen_read_byte(client, N2DM_REG_CTRL4_REG, databuf))
		{
			GSE_ERR("read N2DM_REG_CTRL4_REG err!\n");
			return N2DM_ERR_I2C;
		}
		else
		{
			GSE_LOG("read  N2DM_REG_CTRL4_REG register: 0x%x\n", databuf[0]);
		}

		databuf[0] &= ~N2DM_WORK_MODE_MASK;//clear 
		databuf[0] |= N2DM_WORK_MODE_HR_SET;
		
		databuf[1] = databuf[0];
		databuf[0] = N2DM_REG_CTRL4_REG; 
		
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("write full scale register err!\n");
			return N2DM_ERR_I2C;
		}
	}
	else
	{
		if(hwmsen_read_byte(client, N2DM_REG_CTRL1_REG, databuf))
		{
			GSE_ERR("read N2DM_REG_CTRL1_REG err!\n");
			return N2DM_ERR_I2C;
		}
		else
		{
			GSE_LOG("read  N2DM_REG_CTRL1_REG register: 0x%x\n", databuf[0]);
		}		
		
		if(N2DM_WORK_NORMAL_MODE == work_mode)
		{
			databuf[0] &= ~N2DM_WORK_MODE_MASK;//clear 
			//databuf[0] |= N2DM_WORK_MODE_NM_SET;
			
			databuf[1] = databuf[0];
			databuf[0] = N2DM_REG_CTRL1_REG; 
		}
		else if(N2DM_WORK_LP_MODE == work_mode)
		{
			databuf[0] &= ~N2DM_WORK_MODE_MASK;//clear 
			databuf[0] |= N2DM_WORK_MODE_LP_SET;
			
			databuf[1] = databuf[0];
			databuf[0] = N2DM_REG_CTRL1_REG; 
		}
		else
		{
			databuf[1] = databuf[0];
			databuf[0] = N2DM_REG_CTRL1_REG; 
		}		
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("write full scale register err!\n");
			return N2DM_ERR_I2C;
		}
	}
	//need to check sensitivity here
	res = n2dm_Update_Sensitivity(client);
	if(res < 0)
	{
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	
	return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int n2dm_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
	GSE_FUN();     
	
	
	if(hwmsen_read_byte(client, N2DM_REG_CTRL4_REG, databuf))
	{
		GSE_ERR("read N2DM_REG_CTRL4_REG err!\n");
		return N2DM_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  N2DM_REG_CTRL4_REG register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~N2DM_ACC_RANGE_MASK;//clear 
	databuf[0] |= acc_fs;
	databuf[0] |= 0x80;				// FAE for BDU enable
	databuf[1] = databuf[0];
	databuf[0] = N2DM_REG_CTRL4_REG; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return N2DM_ERR_I2C;
	}
	
	
	obj->full_scale = acc_fs;	

	if(hwmsen_read_byte(client, N2DM_REG_CTRL1_REG, databuf))
	{
		GSE_ERR("read N2DM_REG_CTRL1_REG err!\n");
		return N2DM_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  N2DM_REG_CTRL1_REG register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~N2DM_ACC_ENABLE_AXIS_MASK;//clear 
	databuf[0] |= N2DM_ACC_ENABLE_AXIS_X | N2DM_ACC_ENABLE_AXIS_Y| N2DM_ACC_ENABLE_AXIS_Z;
	
	databuf[1] = databuf[0];
	databuf[0] = N2DM_REG_CTRL1_REG; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return N2DM_ERR_I2C;
	}
	
	//need to check sensitivity here
	res = n2dm_Update_Sensitivity(client);
	if(res < 0)
	{
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	
	return N2DM_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
// set the acc sample rate
static int n2dm_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    

	if(hwmsen_read_byte(client, N2DM_REG_CTRL1_REG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return N2DM_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~N2DM_ACC_ODR_MASK;//clear 
	databuf[0] |= sample_rate;
	
	databuf[1] = databuf[0];
	databuf[0] = N2DM_REG_CTRL1_REG; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write sample rate register err!\n");
		return N2DM_ERR_I2C;
	}

	return N2DM_SUCCESS;    
}


/*----------------------------------------------------------------------------*/
static int n2dm_ReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct n2dm_i2c_data *obj = (struct n2dm_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[N2DM_ACC_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = n2dm_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on n2dm error %d!\n", res);
		}
		msleep(20);
	}

	res = N2DM_ReadAccRawData(client, obj->data);
	if(res < 0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -4;
	}
	else
	{
	#if 1
        GSE_LOG("@@## RawData1 obj->data:%d %d %d\n", obj->data[N2DM_AXIS_X], obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);

		obj->data[N2DM_AXIS_X] = (s64)(obj->data[N2DM_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(16*1000); //default high resolution, 16
		obj->data[N2DM_AXIS_Y] = (s64)(obj->data[N2DM_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(16*1000);
		obj->data[N2DM_AXIS_Z] = (s64)(obj->data[N2DM_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(16*1000);
		
        GSE_LOG("@@## RawData2 obj->data:%d %d %d\n", obj->data[N2DM_AXIS_X], obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);
        GSE_LOG("@@## cali_sw cali_sw->data:%d %d %d\n", obj->cali_sw[N2DM_AXIS_X], obj->cali_sw[N2DM_AXIS_Y], obj->cali_sw[N2DM_AXIS_Z]);

		obj->data[N2DM_AXIS_X] += obj->cali_sw[N2DM_AXIS_X];
		obj->data[N2DM_AXIS_Y] += obj->cali_sw[N2DM_AXIS_Y];
		obj->data[N2DM_AXIS_Z] += obj->cali_sw[N2DM_AXIS_Z];
		
        GSE_LOG("@@## RawData3 obj->data:%d %d %d\n", obj->data[N2DM_AXIS_X], obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);
		/*remap coordinate*/
		acc[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->data[N2DM_AXIS_X];
		acc[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->data[N2DM_AXIS_Y];
		acc[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->data[N2DM_AXIS_Z];

        GSE_LOG("@@## remap acc:%d %d %d\n", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);
		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);

		//Out put the mg
		/*
		acc[N2DM_AXIS_X] = acc[N2DM_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[N2DM_AXIS_Y] = acc[N2DM_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[N2DM_AXIS_Z] = acc[N2DM_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		*/
	#endif

// begin baibo@yulong for acc_calibration 20150906
#ifdef YULONG_ACC_CALIBRATION
		acc_data[N2DM_AXIS_X] = acc[N2DM_AXIS_X];
		acc_data[N2DM_AXIS_Y] = acc[N2DM_AXIS_Y];
		acc_data[N2DM_AXIS_Z] = acc[N2DM_AXIS_Z];
#endif
// end baibo@yulong for acc_calibration 20150906

		sprintf(buf, "%04x %04x %04x", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);
		GSE_LOG("obj->data:%04x %04x %04x\n", obj->data[N2DM_AXIS_X], obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);
		GSE_LOG("acc:%04x %04x %04x\n", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);
		
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			//n2dm_dumpReg(client);
		}
	}
	
	return 0;
}
static int N2DM_ReadAccRawData(struct i2c_client *client, s16 data[N2DM_ACC_AXES_NUM])
{
	//struct n2dm_i2c_data *priv = i2c_get_clientdata(client);        

	int err = 0;
	char databuf[6] = {0};

	if(NULL == client)
	{
		err = -EINVAL;
	}	
	else
	{
		if(hwmsen_read_block(client, I2C_AUTO_INCREMENT | N2DM_REG_OUT_X_L, databuf, 6))
		{
			GSE_ERR("N2DM read acc data  error\n");
			return -2;
		}
		else
		{	
			data[N2DM_AXIS_X] = (s16)((databuf[N2DM_AXIS_X*2+1] << 8) | (databuf[N2DM_AXIS_X*2]));
			data[N2DM_AXIS_Y] = (s16)((databuf[N2DM_AXIS_Y*2+1] << 8) | (databuf[N2DM_AXIS_Y*2]));
			data[N2DM_AXIS_Z] = (s16)((databuf[N2DM_AXIS_Z*2+1] << 8) | (databuf[N2DM_AXIS_Z*2]));	
		}      
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int n2dm_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "n2dm Chip");
	return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = n2dm_i2c_client;
	char strbuf[N2DM_BUFSIZE];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
//begin baibo@yulong.com 201509017 for debug cali
#ifdef DEBUG_CALIBRATION
	GSE_ERR("@@## debug calibration interface\n");
    n2dm_ResetCalibration(client);
	yulong_accel_Calibration(client, strbuf, N2DM_BUFSIZE, 20, (unsigned long)&acc_cal_data);
#endif
//end baibo@yulong.com 201509017 for debug cali

	n2dm_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = n2dm_i2c_client;
	char strbuf[N2DM_BUFSIZE];
	int x,y,z;
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	n2dm_ReadAccData(client, strbuf, N2DM_BUFSIZE);
	sscanf(strbuf, "%x %x %x", &x, &y, &z);	
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x,y,z);            
}
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = n2dm_i2c_client;
	//char strbuf[N2DM_BUFSIZE];
	s16 data[N2DM_ACC_AXES_NUM] = {0};
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	//n2dm_ReadAccData(client, strbuf, N2DM_BUFSIZE);
	N2DM_ReadAccRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0],data[1],data[2]);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct n2dm_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct n2dm_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	
	return count;    
}

// begin baibo@yulong.com 20150909 for print&write regs
static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	u8 regdata;
	int count=0;
	int i=0;
	struct n2dm_i2c_data *obj = obj_i2c_data;

	if (NULL == obj)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	GSE_ERR("@@##   show_chipinit_value!\n");
	for(i=0; i<64; i++)
	{
	    if(hwmsen_read_byte(obj->client, 0x00+i, &regdata))
	    {
		    GSE_ERR("@@##   read acc register err!\n");
		    return 0;
	    }
	    else
	    {
		    count+=snprintf(buf+count, PAGE_SIZE-count, "[0x%x] = (0x%x)\n",0x00+i,regdata);
	    }
	}

	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
#if 0
	struct n2dm_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	n2dm_init_client(obj->client, true);
	n2dm_dumpReg(obj->client);
	
	return count;  
#else
	int reg_addr = 0;
    int reg_val = 0;
    struct n2dm_i2c_data *obj = obj_i2c_data;
	if(!obj)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &reg_addr, &reg_val))
	{
		GSE_ERR("@@##	invalid format: %s\n", buf);
		return 0;
	}
	
    GSE_LOG("@@##   reg_addr=0x%x, reg_val=0x%x\n", reg_addr, reg_val);	
	if (hwmsen_write_byte(obj->client,reg_addr,reg_val))
    {
		GSE_ERR("@@##	i2c write regs error!!\n");
		return 0;   
    }

	return count;  

#endif
}
// end baibo@yulong.com 20150909 for print&write regs

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct n2dm_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->sensitivity, obj->hw->power_id, obj->hw->power_vol);   
	n2dm_dumpReg(obj->client);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct n2dm_i2c_data *data = i2c_get_clientdata(n2dm_i2c_client);

	if(NULL == n2dm_i2c_client)
	{
		GSE_ERR("this_client IS NULL !\n");
		return -1;
	}

	if(NULL == data)
	{
		printk(KERN_ERR "n2dm_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct n2dm_i2c_data *data = i2c_get_clientdata(n2dm_i2c_client);

	if(NULL == n2dm_i2c_client)
	{
		GSE_ERR("this_client IS NULL !\n");
		return -1;
	}

	if(NULL == data)
	{
		printk(KERN_ERR "n2dm_i2c_data is null!!\n");
		return -1;
	}

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensorrawdata,           S_IRUGO, show_sensorrawdata_value,    NULL);

static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(chipinit,      S_IWUSR | S_IRUGO, show_chipinit_value,         store_chipinit_value);

static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *n2dm_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/	
	&driver_attr_sensorrawdata,   /*dump sensor raw data*/	
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,  
	&driver_attr_chipinit,
	&driver_attr_layout,
};
/*----------------------------------------------------------------------------*/
static int n2dm_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(n2dm_attr_list)/sizeof(n2dm_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver,  n2dm_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n",  n2dm_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( n2dm_attr_list)/sizeof( n2dm_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  n2dm_attr_list[idx]);
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int n2dm_init_client(struct i2c_client *client, bool enable)
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	GSE_FUN();	
    GSE_LOG(" n2dm addr %x!\n",client->addr);
	res = n2dm_CheckDeviceID(client);
	if(res != N2DM_SUCCESS)
	{
		return res;
	}

	res = n2dm_SetPowerMode(client, enable);
	if(res != N2DM_SUCCESS)
	{
		return res;
	}
	
	res = n2dm_SetFullScale(client,N2DM_ACC_RANGE_2g);//we have only this choice
	if(res != N2DM_SUCCESS) 
	{
		return res;
	}

	// 
	res = n2dm_SetSampleRate(client, N2DM_ACC_ODR_100HZ);
	if(res != N2DM_SUCCESS ) 
	{
		return res;
	}
	
	res = n2dm_SetWorkMode(client, N2DM_WORK_HR_MODE); //default high resoltion mode
	if(res != N2DM_SUCCESS ) 
	{
		return res;
	}

	GSE_LOG("n2dm_init_client OK!\n");
		
	
#ifdef CONFIG_N2DM_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/

// begin baibo@yulong for acc_calibration 20150906
#ifdef YULONG_ACC_CALIBRATION
static int yulong_accel_Calibration(struct i2c_client *client, char *buf, int bufsize,int count,unsigned long cal_arg)
{
    int ret = 0;
    char i;
    int cali[3];
    char strbuf[N2DM_BUFSIZE];
    long xavg = 0, yavg = 0, zavg = 0;
    char tempbuf[NAND_FLASH_WR_RD_SIZE];
    struct acc_offset acc_cal_data;
    struct acc_offset *acc_cal_ptr = &acc_cal_data;

    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);

    N2DM_power(obj->hw, 1);
    msleep(50);
    n2dm_SetPowerMode(obj->client, true);

    for (i = 0; i < count; i++)
    {
    	n2dm_ReadAccData(client, strbuf, N2DM_BUFSIZE);

        xavg += acc_data[N2DM_AXIS_X];
        yavg += acc_data[N2DM_AXIS_Y];
        zavg += acc_data[N2DM_AXIS_Z];
    }

    printk("@@## (%d),xavg is %ld,Yavg is %ld,Zavg is %ld\n",__LINE__,xavg,yavg,zavg);  
    cali[N2DM_AXIS_X] = xavg/count;
    cali[N2DM_AXIS_Y] = yavg/count;
    cali[N2DM_AXIS_Z] = zavg/count;
    printk("@@## (%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[N2DM_AXIS_X],cali[N2DM_AXIS_Y],cali[N2DM_AXIS_Z]);
    cali[N2DM_AXIS_X] = 0-cali[N2DM_AXIS_X];
    cali[N2DM_AXIS_Y] = 0-cali[N2DM_AXIS_Y];
    cali[N2DM_AXIS_Z] = 9807-cali[N2DM_AXIS_Z];

    //cali[N2DM_AXIS_X] = cali[N2DM_AXIS_X] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    //cali[N2DM_AXIS_Y] = cali[N2DM_AXIS_Y] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    //cali[N2DM_AXIS_Z] = cali[N2DM_AXIS_Z] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    printk("@@## (%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[N2DM_AXIS_X],cali[N2DM_AXIS_Y],cali[N2DM_AXIS_Z]);

    ret = n2dm_WriteCalibration(client, cali);

#ifdef DEBUG_CALIBRATION
    nv_data[N2DM_AXIS_X] = obj->cali_sw[N2DM_AXIS_X];
    nv_data[N2DM_AXIS_Y] = obj->cali_sw[N2DM_AXIS_Y];
    nv_data[N2DM_AXIS_Z] = obj->cali_sw[N2DM_AXIS_Z];
#endif

   printk("@@## obj->cali_sw[N2DM_AXIS_X]=%d,obj->cali_sw[N2DM_AXIS_Y]=%d,obj->cali_sw[N2DM_AXIS_Z]=%d\n",
            obj->cali_sw[N2DM_AXIS_X],obj->cali_sw[N2DM_AXIS_Y],obj->cali_sw[N2DM_AXIS_Z]); 

   ((struct acc_offset *)acc_cal_ptr)->x = obj->cali_sw[N2DM_AXIS_X];
   ((struct acc_offset *)acc_cal_ptr)->y = obj->cali_sw[N2DM_AXIS_Y];
   ((struct acc_offset *)acc_cal_ptr)->z = obj->cali_sw[N2DM_AXIS_Z];
   ((struct acc_offset *)acc_cal_ptr)->key = ret ? 2 : 1;

   //printk(KERN_INFO "%s write: x = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
   //printk(KERN_INFO "%s write: y = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
   //printk(KERN_INFO "%s write: z = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->z);
   //printk(KERN_INFO "%s write: key = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->key);
   memset(tempbuf, 0, sizeof(tempbuf));

   tempbuf[0]=0x01;
   tempbuf[1] =  (obj->cali_sw[N2DM_AXIS_X] & 0xff00) >> 8;
   tempbuf[2] = obj->cali_sw[N2DM_AXIS_X] & 0x00ff;
   tempbuf[3] = (obj->cali_sw[N2DM_AXIS_Y] & 0xff00) >> 8;
   tempbuf[4] = obj->cali_sw[N2DM_AXIS_Y] & 0x00ff;
   tempbuf[5] = (obj->cali_sw[N2DM_AXIS_Z] & 0xff00) >> 8;
   tempbuf[6] = obj->cali_sw[N2DM_AXIS_Z] & 0x00ff;

   if(copy_to_user((struct acc_offset *)cal_arg, acc_cal_ptr, sizeof(acc_cal_data)))
   {
       printk("%s:  Calibrate copy_to_user failed!\n", __func__);          
   }

   //  printk("(%s) TEMPBUF is (%d),(%d),(%d),(%d),(%d),(%d),(%d)\n",
   //      __LINE__,tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6] );
   //  printk("(%s) TEMPBUF is (%x),(%x),(%x),(%x),(%x),(%x),(%x)\n",
   //      __LINE__,tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6] );

   if(write_acc_ps_cal_data_to_flash(16, tempbuf, NAND_FLASH_WR_RD_SIZE)<0)
       printk("Create ACC  calibration file error!!");
   else
       printk("Create ACC  calibration file Success!!");

   return ret;

}

static int yulong_acc_ReadCalibration(struct i2c_client *client)
{
    char tempbuf[NAND_FLASH_WR_RD_SIZE] = {0};

    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);

    printk("@@##	yl_sensor_debug in yulong_acc_ReadCalibration!\n");

    if(read_acc_ps_cal_data_from_flash(16, tempbuf, NAND_FLASH_WR_RD_SIZE)<0)
        printk("ACC use Default CALI , cali_sw[N2DM_AXIS_X] = %d , cali_sw[N2DM_AXIS_Y] = %d, cali_sw[N2DM_AXIS_Z] = %d\n",
        obj->cali_sw[N2DM_AXIS_X],obj->cali_sw[N2DM_AXIS_Y],obj->cali_sw[N2DM_AXIS_Z]);
    else{
        obj->cali_sw[N2DM_AXIS_X] = (s16)((tempbuf[1]<<8)|tempbuf[2]);
        obj->cali_sw[N2DM_AXIS_Y] = (s16)((tempbuf[3]<<8)|tempbuf[4]);
        obj->cali_sw[N2DM_AXIS_Z] = (s16)((tempbuf[5]<<8)|tempbuf[6]);

        printk("ACC use yulong CALI ,tempbuf is %d,%d,%d,%d,%d,%d,%d ",
            tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6]);

        printk("ACC use yulong CALI , cali_sw[N2DM_AXIS_X] = %d , cali_sw[N2DM_AXIS_Y] = %d, cali_sw[N2DM_AXIS_Z] = %d\n",
            obj->cali_sw[N2DM_AXIS_X],obj->cali_sw[N2DM_AXIS_Y],obj->cali_sw[N2DM_AXIS_Z]);
    }
    return 0;
}

#endif
// end baibo@yulong for acc_calibration 20150906


/*----------------------------------------------------------------------------*/
#ifdef N2DM_NEW_ARCH
static int n2dm_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
	
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int n2dm_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct n2dm_i2c_data *priv = obj_i2c_data;
#ifdef YULONG_ACC_CALIBRATION
	static uint8_t n2dm_acc_enable_flag = 0;
#endif	

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

#ifdef YULONG_ACC_CALIBRATION
	if (0 == n2dm_acc_enable_flag)
	{
		GSE_LOG("@@## yulong_acc_ReadCalibration\n");
		n2dm_acc_enable_flag = 1;
		yulong_acc_ReadCalibration(priv->client);
#ifdef DEBUG_CALIBRATION
						priv->cali_sw[N2DM_AXIS_X] = nv_data[N2DM_AXIS_X];
						priv->cali_sw[N2DM_AXIS_Y] = nv_data[N2DM_AXIS_Y];
						priv->cali_sw[N2DM_AXIS_Z] = nv_data[N2DM_AXIS_Z];
#endif
	}
#endif

	if(value == 1)
	{
		enable_status = true;
	}
	else
	{
		enable_status = false;
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
	
	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		
		err = n2dm_SetPowerMode( priv->client, enable_status);					
	}

    GSE_LOG("n2dm_enable_nodata OK!\n");
    return err;
}

static int n2dm_set_delay(u64 ns)
{
    int value =0;
	int err = 0;
	int sample_delay;
	struct n2dm_i2c_data *priv = obj_i2c_data;

    value = (int)ns/1000/1000;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	
					
	if(value <= 5)
	{
		sample_delay = N2DM_ACC_ODR_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = N2DM_ACC_ODR_100HZ;
	}
	else
	{
		sample_delay = N2DM_ACC_ODR_50HZ;
	}
				
	err = n2dm_SetSampleRate(priv->client, sample_delay);
	if(err != N2DM_SUCCESS ) 
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{					
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[N2DM_AXIS_X] = 0;
		priv->fir.sum[N2DM_AXIS_Y] = 0;
		priv->fir.sum[N2DM_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

    GSE_LOG("n2dm_set_delay (%d), chip only use 1024HZ \n",value);
    return 0;
}

static int n2dm_get_data(int* x ,int* y,int* z, int* status)
{
    char buff[N2DM_BUFSIZE];
	struct n2dm_i2c_data *priv = obj_i2c_data;
	
	GSE_LOG("%s (%d),  \n",__FUNCTION__,__LINE__);

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	memset(buff, 0, sizeof(buff));
	n2dm_ReadAccData(priv->client, buff, N2DM_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;				

    return 0;
}


#else
static int n2dm_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct n2dm_i2c_data *priv = (struct n2dm_i2c_data*)self;

	hwm_sensor_data* gsensor_data;
	char buff[N2DM_BUFSIZE];

	//begin baibo@yulong.com for read cali data when first enable sensor 20150906
#ifdef YULONG_ACC_CALIBRATION
	static uint8_t n2dm_acc_enable_flag = 0;
#endif
	//end baibo@yulong.com for read cali data when first enable sensor 20150906
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = N2DM_ACC_ODR_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = N2DM_ACC_ODR_100HZ;
				}
				else
				{
					sample_delay = N2DM_ACC_ODR_50HZ;
				}
				
				
				n2dm_SetSampleRate(priv->client, sample_delay);
				if(err != N2DM_SUCCESS ) 
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[N2DM_AXIS_X] = 0;
					priv->fir.sum[N2DM_AXIS_Y] = 0;
					priv->fir.sum[N2DM_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{			    
				value = *(int *)buff_in;
//begin baibo@yulong.com for read calibration data when first enable sensor 20150906
#ifdef YULONG_ACC_CALIBRATION
                if (0 == n2dm_acc_enable_flag)
                {
                    GSE_LOG("@@## yulong_acc_ReadCalibration\n");
                    n2dm_acc_enable_flag = 1;
                    yulong_acc_ReadCalibration(priv->client);
#ifdef DEBUG_CALIBRATION
                    priv->cali_sw[N2DM_AXIS_X] = nv_data[N2DM_AXIS_X];
                    priv->cali_sw[N2DM_AXIS_Y] = nv_data[N2DM_AXIS_Y];
                    priv->cali_sw[N2DM_AXIS_Z] = nv_data[N2DM_AXIS_Z];
#endif
                }
#endif
//end baibo@yulong.com for read calibration data when first enable sensor 20150906
				if(value == 1)
				{
					enable_status = true;
				}
				else
				{
					enable_status = false;
				}
				GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
				
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					
					err = n2dm_SetPowerMode( priv->client, enable_status);					
				}

			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				n2dm_ReadAccData(priv->client, buff, N2DM_BUFSIZE);
				
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int n2dm_open(struct inode *inode, struct file *file)
{
	file->private_data = n2dm_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int n2dm_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long n2dm_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct n2dm_i2c_data *obj = (struct n2dm_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[N2DM_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
// begin baibo@yulong for acc_calibration 20150906
#ifdef YULONG_ACC_CALIBRATION
		case ACC_CALIBRATE:
			printk("@@## acc_calibrate in %s,%d\n",__func__,__LINE__);

			n2dm_ResetCalibration(client);
            err = yulong_accel_Calibration(client, strbuf, N2DM_BUFSIZE, 20, arg);
            if(0 > err)
            {
                GSE_ERR("@@##	accel calibration err is %d!!\n",err);
                err = -EINVAL;
            }
            
		  break;
#endif
// end baibo@yulong for acc_calibration 20150906
		case GSENSOR_IOCTL_INIT:			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			
			n2dm_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			n2dm_ReadAccData(client, strbuf, N2DM_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
	#if 0			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}	
	#endif
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
	#if 0		
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}	
	#endif
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			N2DM_ReadAccRawData(client, (s16 *)strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
		#if 0
			cali[N2DM_AXIS_X] = (s64)(sensor_data.x) * 1000/(obj->sensitivity*GRAVITY_EARTH_1000); //NTC
			cali[N2DM_AXIS_Y] = (s64)(sensor_data.y) * 1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			cali[N2DM_AXIS_Z] = (s64)(sensor_data.z) * 1000/(obj->sensitivity*GRAVITY_EARTH_1000);
		#else
			cali[N2DM_AXIS_X] = (s64)(sensor_data.x);
			cali[N2DM_AXIS_Y] = (s64)(sensor_data.y);	
			cali[N2DM_AXIS_Z] = (s64)(sensor_data.z);	
		#endif
				err = n2dm_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = n2dm_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			err = n2dm_ReadCalibration(client, cali);
			if(err)
			{
				break;
			}
					
		#if 0
			sensor_data.x = (s64)(cali[N2DM_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000); //NTC
			sensor_data.y = (s64)(cali[N2DM_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000);
			sensor_data.z = (s64)(cali[N2DM_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000);
		#else
			sensor_data.x = (s64)(cali[N2DM_AXIS_X]);
			sensor_data.y = (s64)(cali[N2DM_AXIS_Y]);
			sensor_data.z = (s64)(cali[N2DM_AXIS_Z]);
		#endif
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}
#ifdef CONFIG_COMPAT
static long n2dm_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations n2dm_acc_fops = {
	.owner = THIS_MODULE,
	.open = n2dm_open,
	.release = n2dm_release,
	.unlocked_ioctl = n2dm_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = n2dm_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice n2dm_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &n2dm_acc_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef CONFIG_HAS_EARLYSUSPEND
#if 1
/*----------------------------------------------------------------------------*/
static int n2dm_acc_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;

	GSE_FUN();
	
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		
		err = n2dm_SetPowerMode(obj->client, false);
		if(err)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		
		sensor_power = false;
		
		N2DM_power(obj->hw, 0);

	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_acc_resume(struct i2c_client *client)
{
	struct n2dm_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	N2DM_power(obj->hw, 1);
	
	err = n2dm_SetPowerMode(obj->client, enable_status);
	if(err)
	{
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return err ;        
	}
	atomic_set(&obj->suspend, 0);  

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void n2dm_early_suspend(struct early_suspend *h) 
{
	struct n2dm_i2c_data *obj = container_of(h, struct n2dm_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
	
	err = n2dm_SetPowerMode(obj->client, false);
	if(err)
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}
	if(err <= 0)
	{
		return;
	}

	sensor_power = false;
	
	N2DM_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void n2dm_late_resume(struct early_suspend *h)
{
	struct n2dm_i2c_data *obj = container_of(h, struct n2dm_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	N2DM_power(obj->hw, 1);
	
	err = n2dm_SetPowerMode(obj->client, enable_status);

	if(err)
	{
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}

/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/

/*static int n2dm_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, N2DM_ACC_DEV_NAME);
	return 0;
}
*/

// begin baibo@yulong for device info 20150906
#ifdef N2DM_DEVICE_INFO
extern int get_device_info(char* buf);
#endif
// end baibo@yulong for device info 20150906

/*start to modify for chip info by qinxinjun@yulong.com in 20160504*/
static struct sensors_classdev sensors_accelerometer_cdev = {
         .name = "accelerometer",
         .vendor = "N2DM",
         .version = 1,
         .handle = SENSORS_ACCELERATION_HANDLE,
         .type = SENSOR_TYPE_ACCELEROMETER,
         .max_range = "100000",
         .resolution = "1.0",
         .sensor_power = "0.1",
         .min_delay = 20000, /* in microseconds */
         .fifo_reserved_event_count = 0,
         .fifo_max_event_count = 0,
         .sensors_enable = NULL,
         .sensors_poll_delay = NULL,
};
/*end to modify for chip info by qinxinjun@yulong.com in 20160504*/


/*----------------------------------------------------------------------------*/
static int n2dm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct n2dm_i2c_data *obj;

#ifdef N2DM_NEW_ARCH
	struct acc_control_path ctl={0};
    struct acc_data_path data={0};
#else
	struct hwmsen_object gyro_sobj;
	struct hwmsen_object acc_sobj;
#endif
	int err = 0;
	//GSE_FUN();
    /*start to modify for chip info by qinxinjun@yulong.com in 20160504*/
    sensors_classdev_register(&client->dev,&sensors_accelerometer_cdev);
    /*end to modify for chip info by qinxinjun@yulong.com in 20160504*/

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct n2dm_i2c_data));

	// begin baibo@yulong for compatible 20150906
	obj->hw = hw;
	// end baibo@yulong for compatible 20150906
	
	atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if(err)
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	n2dm_i2c_client = new_client;	
	err = n2dm_init_client(new_client, false);
	if(err)
	{
		goto exit_init_failed;
	}
	
	err = misc_register(&n2dm_acc_device);
	if(err)
	{
		GSE_ERR("misc_register misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
	err = n2dm_create_attr(&(n2dm_init_info.platform_diver_addr->driver));
#else
	err = n2dm_create_attr(&n2dm_driver.driver);
#endif
	if(err)
	{
		GSE_ERR("n2dm create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}	

#ifdef N2DM_NEW_ARCH
    ctl.open_report_data= n2dm_open_report_data;
    ctl.enable_nodata = n2dm_enable_nodata;
    ctl.set_delay  = n2dm_set_delay;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;

    err = acc_register_control_path(&ctl);
    if(err)
    {
         GSE_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = n2dm_get_data;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if(err)
    {
        GSE_ERR("register acc data path err= %d\n", err);
        goto exit_kfree;
    }

#else
	acc_sobj.self = obj;
    acc_sobj.polling = 1;
    acc_sobj.sensor_operate = n2dm_operate;
	err = hwmsen_attach(ID_ACCELEROMETER, &acc_sobj);
	if(err)
	{
		GSE_ERR("hwmsen_attach Accelerometer fail = %d\n", err);
		goto exit_kfree;
	}
#endif	

//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = n2dm_early_suspend,
	obj->early_drv.resume   = n2dm_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 
#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
	n2dm_init_flag = 0;
#endif

// begin baibo@yulong for device info 20150906
#ifdef N2DM_DEVICE_INFO
	get_device_info(" ACC: n2dm ST\n");
#endif
// end baibo@yulong for device info 20150906
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	
	misc_deregister(&n2dm_acc_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
	n2dm_init_flag = -1;
#endif
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int n2dm_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))	
	err = n2dm_delete_attr(&(n2dm_init_info.platform_diver_addr->driver));
#else
	err = n2dm_delete_attr(&n2dm_driver.driver);
#endif
	if(err)
	{
		GSE_ERR("n2dm_i2c_remove fail: %d\n", err);
	}

	err = misc_deregister(&n2dm_acc_device);
	if(err)
	{
		GSE_ERR("misc_deregister n2dm_acc_device fail: %d\n", err);
	}

	n2dm_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
#if (defined(N2DM_NEW_ARCH) || defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))
static int n2dm_local_init(void)
{
	// begin baibo@yulong for compatible 20150906
	//struct acc_hw *accel_hw = n2dm_get_cust_acc_hw();
	// end baibo@yulong for compatible 20150906
	//GSE_FUN();

	N2DM_power(hw, 1);
	
	if(i2c_add_driver(&n2dm_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(n2dm_init_flag == -1)
	{
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	return 0;
}
static int n2dm_local_uninit(void)
{
	// begin baibo@yulong for compatible 20150906
	//struct acc_hw *accel_hw = n2dm_get_cust_acc_hw();
	// end baibo@yulong for compatible 20150906
    //GSE_FUN();    
    N2DM_power(hw, 0);  	
    i2c_del_driver(&n2dm_i2c_driver);
    return 0;
}
#else
static int n2dm_probe(struct platform_device *pdev) 
{
	// begin baibo@yulong for compatible 20150906
	//struct acc_hw *accel_hw = n2dm_get_cust_acc_hw();
	// end baibo@yulong for compatible 20150906
	GSE_FUN();

	N2DM_power(hw, 1);
	
	if(i2c_add_driver(&n2dm_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int n2dm_remove(struct platform_device *pdev)
{
	// begin baibo@yulong for compatible 20150906
	//struct acc_hw *accel_hw = n2dm_get_cust_acc_hw();
	// end baibo@yulong for compatible 20150906
    //GSE_FUN();    
    N2DM_power(hw, 0);  	
    i2c_del_driver(&n2dm_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor", },
	{},
};
#endif

static struct platform_driver n2dm_driver = {
	.probe      = n2dm_probe,
	.remove     = n2dm_remove,    
	.driver     = {
			.name  = "gsensor",
		//	.owner	= THIS_MODULE,
	#ifdef CONFIG_OF
			.of_match_table = gsensor_of_match,
	#endif

	}
};

#endif
/*----------------------------------------------------------------------------*/
static int __init n2dm_init(void)
{
	const char *name = "mediatek,n2dm";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&n2dm_init_info);
	return 0;  
}

/*----------------------------------------------------------------------------*/
static void __exit n2dm_exit(void)
{
	GSE_FUN();
#if (!defined(N2DM_NEW_ARCH) && !defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER))	
	platform_driver_unregister(&n2dm_driver);
#endif

}
/*----------------------------------------------------------------------------*/
module_init(n2dm_init);
module_exit(n2dm_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("N2DM Accelerometer driver");
MODULE_AUTHOR("darren.han@st.com");

/*----------------------------------------------------------------- N2DM ------------------------------------------------------------------*/
