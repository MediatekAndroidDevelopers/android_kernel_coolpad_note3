/* BMA250 accel sensor driver
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
 * Modifications:
 * ===============================
 * wangyufei@yulong.com add gsensor calibration on 20150416
 * huyue2@yulong.com porting to Android 6.0 on 2016/02/22
 */

#include "cust_acc.h"
#include "accel.h"
#include "bma250.h"
#include <linux/sensors.h>/*add for chip info by qinxinjun@yulong.com in 20160504*/
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA250 250
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_BMA150_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define BMA250_AXIS_X          0
#define BMA250_AXIS_Y          1
#define BMA250_AXIS_Z          2
#define BMA250_AXES_NUM        3
#define BMA250_DATA_LEN        6
#define BMA250_DEV_NAME        "BMA250"
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

// add by wangyufei@yulong.com modify for gsensor calibration begin 20150416 
#define NAND_FLASH_WR_RD_SIZE 10
//#include <linux/wakelock.h>
// add by wangyufei@yulong.com modify for gsensor calibration end 20150416 

static const struct i2c_device_id bma250_i2c_id[] = {{BMA250_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_bma250={ I2C_BOARD_INFO("BMA250", (0x30>>1))};

typedef struct {
	int x;
	int y;
	int z;
} SENSOR_DATA;

struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
        {.compatible = "mediatek,gsensor"},
        {},
};
#endif


/*the adapter id will be available in customization*/
//static unsigned short bma250_force[] = {0x00, BMA250_I2C_SLAVE_WRITE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const bma250_forces[] = { bma250_force, NULL };
//static struct i2c_client_address_data bma250_addr_data = { .forces = bma250_forces,};

/*----------------------------------------------------------------------------*/
static int bma250_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma250_i2c_remove(struct i2c_client *client);
static int bma250_suspend(struct i2c_client *client, pm_message_t msg);
static int bma250_resume(struct i2c_client *client);
//static int bma250_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

// add by wangyufei@yulong.com modify for gsensor calibration begin 20150416 

#define CAL_FUZZ 380                //wangyufei@yulong.com modify for acc calibration at 20141205
#ifdef CAL_MODE
s16 acc_data_b[3];
//static int match_sensor_flag = 0;
extern int yl_params_kernel_write(char *buf, size_t count, int ver);
extern int yl_params_kernel_read(char *buf, size_t count, int ver);
extern int read_acc_ps_cal_data_from_flash(unsigned int offset, char* output, int counts);
extern int write_acc_ps_cal_data_to_flash(unsigned int offset, char* input, int counts);
#endif
// add by wangyufei@yulong.com modify for gsensor calibration begin 20150416 


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
    s16 raw[C_MAX_FIR_LENGTH][BMA250_AXES_NUM];
    int sum[BMA250_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct bma250_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[BMA250_AXES_NUM+1];

    /*data*/
    s8                      offset[BMA250_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA250_AXES_NUM+1];

#if defined(CONFIG_BMA250_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif

#if 0
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver bma250_i2c_driver = {
    .driver = {
 //       .owner          = THIS_MODULE,
        .name           = BMA250_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = accel_of_match,
#endif
    },
	.probe      		= bma250_i2c_probe,
	.remove    			= bma250_i2c_remove,
//	.detect				= bma250_i2c_detect,
//#if !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend            = bma250_suspend,
    .resume             = bma250_resume,
//#endif
	.id_table = bma250_i2c_id,
//	.address_data = &bma250_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma250_i2c_client = NULL;
//static struct platform_driver bma250_gsensor_driver;
static struct bma250_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
//static char selftestRes[8]= {0};
/*add by mayulong 20130923 begin */
// #if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
static int bma250_init_flag = -1;
static int bma250_local_init(void);
static int bma250_remove(void);
static struct acc_init_info bma250_init_info = {
        .name = "bma250",
        .init = bma250_local_init,
        .uninit = bma250_remove,
};
#endif
/*add by mayulong 20130923 end */


/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)// modify baibo@yulong.com 20150916 for debug
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)// modify baibo@yulong.com 20150916 for debug
/*----------------------------------------------------------------------------*/
static struct data_resolution bma250_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 3, 9}, 256},   // dataformat +/-2g  in 8-bit resolution;  { 15, 6} = 15.6= (2*2*1000)/(2^8);  64 = (2^8)/(2*2)          
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma250_offset_resolution = {{3, 9}, 256};

/*--------------------BMA250 power control function----------------------------------*/
static void BMA250_power(struct acc_hw *hw, unsigned int on) 
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
			if(!hwPowerOn(hw->power_id, hw->power_vol, "BMA250"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "BMA250"))
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
static int BMA250_SetDataResolution(struct bma250_i2c_data *obj)
{

/*set g sensor dataresolution here*/

/*BMA250 only can set to 10-bit dataresolution, so do nothing in bma250 driver here*/

/*end of set dataresolution*/


 
 /*we set measure range from -2g to +2g in BMA150_SetDataFormat(client, BMA150_RANGE_2G), 
                                                    and set 10-bit dataresolution BMA150_SetDataResolution()*/
                                                    
 /*so bma250_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

 	obj->reso = &bma250_data_resolution[0];
	return 0;
	
/*if you changed the measure range, for example call: BMA250_SetDataFormat(client, BMA150_RANGE_4G), 
you must set the right value to bma250_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadData(struct i2c_client *client, s16 data[BMA250_AXES_NUM])
{
#ifdef CONFIG_BMA250_LOWPASS
	struct bma250_i2c_data *priv = i2c_get_clientdata(client);        
#endif
	u8 addr = BMA250_REG_DATAXLOW;
	u8 buf[BMA250_DATA_LEN] = {0};
	int err = 0;
	int i;

	err = hwmsen_read_block(client, addr, buf, 0x06);

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if(err)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[BMA250_AXIS_X] = (s16)((buf[BMA250_AXIS_X*2] >> 6) |
		         (buf[BMA250_AXIS_X*2+1] << 2));
		data[BMA250_AXIS_Y] = (s16)((buf[BMA250_AXIS_Y*2] >> 6) |
		         (buf[BMA250_AXIS_Y*2+1] << 2));
		data[BMA250_AXIS_Z] = (s16)((buf[BMA250_AXIS_Z*2] >> 6) |
		         (buf[BMA250_AXIS_Z*2+1] << 2));

		for(i=0;i<3;i++)				
		{								//because the data is store in binary complement number formation in computer system
			if ( data[i] == 0x0200 )	//so we want to calculate actual number here
				data[i]= -512;			//10bit resolution, 512= 2^(10-1)
			else if ( data[i] & 0x0200 )//transfor format
			{							//printk("data 0 step %x \n",data[i]);
				data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
				data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
				data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
				data[i] = -data[i];		
			}
		}	

		if(1)//(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[BMA250_AXIS_X], data[BMA250_AXIS_Y], data[BMA250_AXIS_Z],
		                               data[BMA250_AXIS_X], data[BMA250_AXIS_Y], data[BMA250_AXIS_Z]);
		}
#ifdef CONFIG_BMA250_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA250_AXIS_X] = data[BMA250_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA250_AXIS_Y] = data[BMA250_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA250_AXIS_Z] = data[BMA250_AXIS_Z];
					priv->fir.sum[BMA250_AXIS_X] += data[BMA250_AXIS_X];
					priv->fir.sum[BMA250_AXIS_Y] += data[BMA250_AXIS_Y];
					priv->fir.sum[BMA250_AXIS_Z] += data[BMA250_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][BMA250_AXIS_X], priv->fir.raw[priv->fir.num][BMA250_AXIS_Y], priv->fir.raw[priv->fir.num][BMA250_AXIS_Z],
							priv->fir.sum[BMA250_AXIS_X], priv->fir.sum[BMA250_AXIS_Y], priv->fir.sum[BMA250_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA250_AXIS_X] -= priv->fir.raw[idx][BMA250_AXIS_X];
					priv->fir.sum[BMA250_AXIS_Y] -= priv->fir.raw[idx][BMA250_AXIS_Y];
					priv->fir.sum[BMA250_AXIS_Z] -= priv->fir.raw[idx][BMA250_AXIS_Z];
					priv->fir.raw[idx][BMA250_AXIS_X] = data[BMA250_AXIS_X];
					priv->fir.raw[idx][BMA250_AXIS_Y] = data[BMA250_AXIS_Y];
					priv->fir.raw[idx][BMA250_AXIS_Z] = data[BMA250_AXIS_Z];
					priv->fir.sum[BMA250_AXIS_X] += data[BMA250_AXIS_X];
					priv->fir.sum[BMA250_AXIS_Y] += data[BMA250_AXIS_Y];
					priv->fir.sum[BMA250_AXIS_Z] += data[BMA250_AXIS_Z];
					priv->fir.idx++;
					data[BMA250_AXIS_X] = priv->fir.sum[BMA250_AXIS_X]/firlen;
					data[BMA250_AXIS_Y] = priv->fir.sum[BMA250_AXIS_Y]/firlen;
					data[BMA250_AXIS_Z] = priv->fir.sum[BMA250_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][BMA250_AXIS_X], priv->fir.raw[idx][BMA250_AXIS_Y], priv->fir.raw[idx][BMA250_AXIS_Z],
						priv->fir.sum[BMA250_AXIS_X], priv->fir.sum[BMA250_AXIS_Y], priv->fir.sum[BMA250_AXIS_Z],
						data[BMA250_AXIS_X], data[BMA250_AXIS_Y], data[BMA250_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadOffset(struct i2c_client *client, s8 ofs[BMA250_AXES_NUM])
{
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
	return 0;
#else
	int err;
	if(err = hwmsen_read_block(client, BMA250_REG_OFSX, ofs, BMA250_AXES_NUM))
	{
		GSE_ERR("error: %d\n", err);
	}
	return err;
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
}
/*----------------------------------------------------------------------------*/
static int BMA250_ResetCalibration(struct i2c_client *client)
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);

#ifdef SW_CALIBRATION
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return 0;

#else
	u8 ofs[4]={0,0,0,0};
	int err;
	if(err = hwmsen_write_block(client, BMA250_REG_OFSX, ofs, 4))
	{
		GSE_ERR("error: %d\n", err);
	}
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
#endif
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadCalibration(struct i2c_client *client, int dat[BMA250_AXES_NUM])
{
    struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	#ifndef SW_CALIBRATION
    int err;
	#endif
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = BMA250_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[BMA250_AXIS_X]] = obj->cvt.sign[BMA250_AXIS_X]*(obj->offset[BMA250_AXIS_X]*mul + obj->cali_sw[BMA250_AXIS_X]);
    dat[obj->cvt.map[BMA250_AXIS_Y]] = obj->cvt.sign[BMA250_AXIS_Y]*(obj->offset[BMA250_AXIS_Y]*mul + obj->cali_sw[BMA250_AXIS_Y]);
    dat[obj->cvt.map[BMA250_AXIS_Z]] = obj->cvt.sign[BMA250_AXIS_Z]*(obj->offset[BMA250_AXIS_Z]*mul + obj->cali_sw[BMA250_AXIS_Z]);                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadCalibrationEx(struct i2c_client *client, int act[BMA250_AXES_NUM], int raw[BMA250_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	#ifndef SW_CALIBRATION
	int err;
	#endif
	int mul;

 

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if(err = BMA250_ReadOffset(client, obj->offset))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
	#endif
	
	raw[BMA250_AXIS_X] = obj->offset[BMA250_AXIS_X]*mul + obj->cali_sw[BMA250_AXIS_X];
	raw[BMA250_AXIS_Y] = obj->offset[BMA250_AXIS_Y]*mul + obj->cali_sw[BMA250_AXIS_Y];
	raw[BMA250_AXIS_Z] = obj->offset[BMA250_AXIS_Z]*mul + obj->cali_sw[BMA250_AXIS_Z];

	act[obj->cvt.map[BMA250_AXIS_X]] = obj->cvt.sign[BMA250_AXIS_X]*raw[BMA250_AXIS_X];
	act[obj->cvt.map[BMA250_AXIS_Y]] = obj->cvt.sign[BMA250_AXIS_Y]*raw[BMA250_AXIS_Y];
	act[obj->cvt.map[BMA250_AXIS_Z]] = obj->cvt.sign[BMA250_AXIS_Z]*raw[BMA250_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA250_WriteCalibration(struct i2c_client *client, int dat[BMA250_AXES_NUM])
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[BMA250_AXES_NUM], raw[BMA250_AXES_NUM];
	#ifndef SW_CALIBRATION
	int lsb = bma250_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
	#endif

	err = BMA250_ReadCalibrationEx(client, cali, raw);
	if(err)	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[BMA250_AXIS_X], raw[BMA250_AXIS_Y], raw[BMA250_AXIS_Z],
		obj->offset[BMA250_AXIS_X], obj->offset[BMA250_AXIS_Y], obj->offset[BMA250_AXIS_Z],
		obj->cali_sw[BMA250_AXIS_X], obj->cali_sw[BMA250_AXIS_Y], obj->cali_sw[BMA250_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA250_AXIS_X] += dat[BMA250_AXIS_X];
	cali[BMA250_AXIS_Y] += dat[BMA250_AXIS_Y];
	cali[BMA250_AXIS_Z] += dat[BMA250_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[BMA250_AXIS_X], dat[BMA250_AXIS_Y], dat[BMA250_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA250_AXIS_X] = obj->cvt.sign[BMA250_AXIS_X]*(cali[obj->cvt.map[BMA250_AXIS_X]]);
	obj->cali_sw[BMA250_AXIS_Y] = obj->cvt.sign[BMA250_AXIS_Y]*(cali[obj->cvt.map[BMA250_AXIS_Y]]);
	obj->cali_sw[BMA250_AXIS_Z] = obj->cvt.sign[BMA250_AXIS_Z]*(cali[obj->cvt.map[BMA250_AXIS_Z]]);	
#else
	obj->offset[BMA250_AXIS_X] = (s8)(obj->cvt.sign[BMA250_AXIS_X]*(cali[obj->cvt.map[BMA250_AXIS_X]])/(divisor));
	obj->offset[BMA250_AXIS_Y] = (s8)(obj->cvt.sign[BMA250_AXIS_Y]*(cali[obj->cvt.map[BMA250_AXIS_Y]])/(divisor));
	obj->offset[BMA250_AXIS_Z] = (s8)(obj->cvt.sign[BMA250_AXIS_Z]*(cali[obj->cvt.map[BMA250_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA250_AXIS_X] = obj->cvt.sign[BMA250_AXIS_X]*(cali[obj->cvt.map[BMA250_AXIS_X]])%(divisor);
	obj->cali_sw[BMA250_AXIS_Y] = obj->cvt.sign[BMA250_AXIS_Y]*(cali[obj->cvt.map[BMA250_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA250_AXIS_Z] = obj->cvt.sign[BMA250_AXIS_Z]*(cali[obj->cvt.map[BMA250_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[BMA250_AXIS_X]*divisor + obj->cali_sw[BMA250_AXIS_X], 
		obj->offset[BMA250_AXIS_Y]*divisor + obj->cali_sw[BMA250_AXIS_Y], 
		obj->offset[BMA250_AXIS_Z]*divisor + obj->cali_sw[BMA250_AXIS_Z], 
		obj->offset[BMA250_AXIS_X], obj->offset[BMA250_AXIS_Y], obj->offset[BMA250_AXIS_Z],
		obj->cali_sw[BMA250_AXIS_X], obj->cali_sw[BMA250_AXIS_Y], obj->cali_sw[BMA250_AXIS_Z]);

	if(err = hwmsen_write_block(obj->client, BMA250_REG_OFSX, obj->offset, BMA250_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
#if 0
static int BMA250_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);    
	databuf[0] = BMA250_REG_DEVID;    

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_BMA250_CheckDeviceID;
	}
	
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_BMA250_CheckDeviceID;
	}
	

	if(databuf[0]!=BMA250_FIXED_DEVID)
	{
		printk("BMA250_CheckDeviceID %d failt!\n ", databuf[0]);
		return BMA250_ERR_IDENTIFICATION;
	}
	else
	{
		printk("BMA250_CheckDeviceID %d pass!\n ", databuf[0]);
	}

	exit_BMA250_CheckDeviceID:
	if (res <= 0)
	{
		return BMA250_ERR_I2C;
	}
	
	return BMA250_SUCCESS;
}
#endif
/*----------------------------------------------------------------------------*/
static int BMA250_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = BMA250_REG_POWER_CTL;
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	
	
	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA250_SUCCESS;
	}

	if(hwmsen_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return BMA250_ERR_I2C;
	}

	
	if(enable == true)
	{
		databuf[0] &= ~BMA250_MEASURE_MODE;
	}
	else
	{
		databuf[0] |= BMA250_MEASURE_MODE;
	}
	databuf[1] = databuf[0];
	databuf[0] = BMA250_REG_POWER_CTL;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return BMA250_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	//GSE_LOG("BMA250_SetPowerMode ok!\n");


	sensor_power = enable;

	mdelay(20);
	
	return BMA250_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA250_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    

	if(hwmsen_read_block(client, BMA250_REG_DATA_FORMAT, databuf, 0x01))
	{
		printk("bma250 read Dataformat failt \n");
		return BMA250_ERR_I2C;
	}

	databuf[0] &= ~BMA250_RANGE_MASK;
	databuf[0] |= dataformat;
	databuf[1] = databuf[0];
	databuf[0] = BMA250_REG_DATA_FORMAT;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return BMA250_ERR_I2C;
	}
	
	//printk("BMA250_SetDataFormat OK! \n");
	

	return BMA250_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA250_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    

	if(hwmsen_read_block(client, BMA250_REG_BW_RATE, databuf, 0x01))
	{
		printk("bma250 read rate failt \n");
		return BMA250_ERR_I2C;
	}

	databuf[0] &= ~BMA250_BW_MASK;
	databuf[0] |= bwrate;
	databuf[1] = databuf[0];
	databuf[0] = BMA250_REG_BW_RATE;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return BMA250_ERR_I2C;
	}
	
	//printk("BMA250_SetBWRate OK! \n");
	
	return BMA250_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA250_SetIntEnable(struct i2c_client *client, u8 intenable)
{
			//u8 databuf[10];
			int res = 0;
		
			res = hwmsen_write_byte(client, BMA250_INT_REG_1, 0x00);
			if(res != BMA250_SUCCESS) 
			{
				return res;
			}
			res = hwmsen_write_byte(client, BMA250_INT_REG_2, 0x00);
			if(res != BMA250_SUCCESS) 
			{
				return res;
			}
			printk("BMA250 disable interrupt ...\n");
		
			/*for disable interrupt function*/
			
			return BMA250_SUCCESS;	  
}

/*----------------------------------------------------------------------------*/
static int bma250_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	printk("bma250_init_client \n");

/* modify by mayulong 20130821 begin */
#if 0
	res = BMA250_CheckDeviceID(client); 
	if(res != BMA250_SUCCESS)
	{
		return res;
	}	
	printk("BMA250_CheckDeviceID ok \n");
#endif
/* modify by mayulong 20130821 end */	
	res = BMA250_SetBWRate(client, BMA250_BW_100HZ);
	if(res != BMA250_SUCCESS ) 
	{
		return res;
	}
	printk("BMA250_SetBWRate OK!\n");
	
	res = BMA250_SetDataFormat(client, BMA250_RANGE_2G);
	if(res != BMA250_SUCCESS) 
	{
		return res;
	}
	printk("BMA250_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


	res = BMA250_SetIntEnable(client, 0x00);        
	if(res != BMA250_SUCCESS)
	{
		return res;
	}
	printk("BMA250 disable interrupt function!\n");

	res = BMA250_SetPowerMode(client, false);
		if(res != BMA250_SUCCESS)
		{
			return res;
		}
		printk("BMA250_SetPowerMode OK!\n");


	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = BMA250_ResetCalibration(client);
		if(res != BMA250_SUCCESS)
		{
			return res;
		}
	}
	printk("bma250_init_client OK!\n");
#ifdef CONFIG_BMA250_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	mdelay(20);

	return BMA250_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
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

	sprintf(buf, "BMA250 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma250_i2c_data *obj = (struct bma250_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA250_AXES_NUM];
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
		res = BMA250_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma250 error %d!\n", res);
		}
	}

	res = BMA250_ReadData(client, obj->data);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA150_AXIS_X],obj->data[BMA150_AXIS_Y],obj->data[BMA150_AXIS_Z]);
		obj->data[BMA250_AXIS_X] += obj->cali_sw[BMA250_AXIS_X];
		obj->data[BMA250_AXIS_Y] += obj->cali_sw[BMA250_AXIS_Y];
		obj->data[BMA250_AXIS_Z] += obj->cali_sw[BMA250_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA150_AXIS_X],obj->cali_sw[BMA150_AXIS_Y],obj->cali_sw[BMA150_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[BMA250_AXIS_X]] = obj->cvt.sign[BMA250_AXIS_X]*obj->data[BMA250_AXIS_X];
		acc[obj->cvt.map[BMA250_AXIS_Y]] = obj->cvt.sign[BMA250_AXIS_Y]*obj->data[BMA250_AXIS_Y];
		acc[obj->cvt.map[BMA250_AXIS_Z]] = obj->cvt.sign[BMA250_AXIS_Z]*obj->data[BMA250_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA150_AXIS_X],obj->cvt.sign[BMA150_AXIS_Y],obj->cvt.sign[BMA150_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA150_AXIS_X], acc[BMA150_AXIS_Y], acc[BMA150_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA150_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA250_AXIS_X] = acc[BMA250_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA250_AXIS_Y] = acc[BMA250_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA250_AXIS_Z] = acc[BMA250_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;	
		
		//add by wangyufei@yulong.com for acc calibration begin at 20150416
		acc_data_b[BMA250_AXIS_X] = acc[BMA250_AXIS_X];
		acc_data_b[BMA250_AXIS_Y] = acc[BMA250_AXIS_Y];
		acc_data_b[BMA250_AXIS_Z] = acc[BMA250_AXIS_Z];
		printk("wangyufei acc_data=%x, y=%x, z=%x",acc[BMA250_AXIS_X],acc[BMA250_AXIS_Y],acc[BMA250_AXIS_Z]);
		//add by wangyufei@yulong.com for acc calibration end at 20150416			
	

		sprintf(buf, "%04x %04x %04x", acc[BMA250_AXIS_X], acc[BMA250_AXIS_Y], acc[BMA250_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA250_ReadRawData(struct i2c_client *client, char *buf)
{
	struct bma250_i2c_data *obj = (struct bma250_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	res = BMA250_ReadData(client, obj->data);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA250_ReadRawData %04x %04x %04x", obj->data[BMA250_AXIS_X], 
			obj->data[BMA250_AXIS_Y], obj->data[BMA250_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma250_i2c_client;
	char strbuf[BMA250_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	BMA250_ReadChipInfo(client, strbuf, BMA250_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
		struct i2c_client *client = bma250_i2c_client;
		char strbuf[BMA250_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		bma250_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif



/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma250_i2c_client;
	char strbuf[BMA250_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA250_ReadSensorData(client, strbuf, BMA250_BUFSIZE);
	//BMA150_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
{
		struct i2c_client *client = bma250_i2c_client;
		char strbuf[BMA250_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//BMA150_ReadSensorData(client, strbuf, BMA150_BUFSIZE);
		BMA250_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma250_i2c_client;
	struct bma250_i2c_data *obj;
	int err, err2, len = 0, mul;
	int tmp[BMA250_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);


	err = BMA250_ReadOffset(client, obj->offset);
	err2 = BMA250_ReadCalibration(client, tmp);
	if(err)
	{
		return -EINVAL;
	}
	else if(err2)
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[BMA250_AXIS_X], obj->offset[BMA250_AXIS_Y], obj->offset[BMA250_AXIS_Z],
			obj->offset[BMA250_AXIS_X], obj->offset[BMA250_AXIS_Y], obj->offset[BMA250_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[BMA250_AXIS_X], obj->cali_sw[BMA250_AXIS_Y], obj->cali_sw[BMA250_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[BMA250_AXIS_X]*mul + obj->cali_sw[BMA250_AXIS_X],
			obj->offset[BMA250_AXIS_Y]*mul + obj->cali_sw[BMA250_AXIS_Y],
			obj->offset[BMA250_AXIS_Z]*mul + obj->cali_sw[BMA250_AXIS_Z],
			tmp[BMA250_AXIS_X], tmp[BMA250_AXIS_Y], tmp[BMA250_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma250_i2c_client;  
	int err, x, y, z;
	int dat[BMA250_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = BMA250_ResetCalibration(client)) > 0 )
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[BMA250_AXIS_X] = x;
		dat[BMA250_AXIS_Y] = y;
		dat[BMA250_AXIS_Z] = z;
		if((err = BMA250_WriteCalibration(client, dat)) > 0)
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA250_LOWPASS
	struct i2c_client *client = bma250_i2c_client;
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA250_AXIS_X], obj->fir.raw[idx][BMA250_AXIS_Y], obj->fir.raw[idx][BMA250_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA250_AXIS_X], obj->fir.sum[BMA250_AXIS_Y], obj->fir.sum[BMA250_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA250_AXIS_X]/len, obj->fir.sum[BMA250_AXIS_Y]/len, obj->fir.sum[BMA250_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_BMA250_LOWPASS
	struct i2c_client *client = bma250_i2c_client;  
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma250_i2c_data *obj = obj_i2c_data;
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
	struct bma250_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct bma250_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if(sensor_power)
		printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return 0;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);


/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma250_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
};
/*----------------------------------------------------------------------------*/
static int bma250_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bma250_attr_list)/sizeof(bma250_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, bma250_attr_list[idx]))>0)
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", bma250_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma250_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bma250_attr_list)/sizeof(bma250_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bma250_attr_list[idx]);
	}
	

	return err;
}


//add by wangyufei@yulong.com modify for gsensor calibration begin 20150415
static int yulong_acc_ReadCalibration(struct i2c_client *client)
{
	char tempbuf[NAND_FLASH_WR_RD_SIZE];
	struct bma250_i2c_data*obj = i2c_get_clientdata(client);  

	printk("WANGYUFEI IN YULONG ACC_READ_CALIBRATION!\n");

	if(read_acc_ps_cal_data_from_flash(16, tempbuf, NAND_FLASH_WR_RD_SIZE)<0)
		printk("Use Default CALI , acc_cali_sw[BMA250_AXIS_X] = %d , acc_cali_sw[BMA250_AXIS_Y] = %d, acc_cali_sw[BMA250_AXIS_Z] = %d\n",
		obj->cali_sw[BMA250_AXIS_X],obj->cali_sw[BMA250_AXIS_Y],obj->cali_sw[BMA250_AXIS_Z]);
	else{
	obj->cali_sw[BMA250_AXIS_X] = (s16)((tempbuf[1]<<8)|tempbuf[2]);
	obj->cali_sw[BMA250_AXIS_Y] = (s16)((tempbuf[3]<<8)|tempbuf[4]);
	obj->cali_sw[BMA250_AXIS_Z] = (s16)((tempbuf[5]<<8)|tempbuf[6]);
			
		printk("Use yulong CALI ,tempbuf is %d,%d,%d,%d,%d,%d,%d ",
			tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6]);
	
		printk("Use yulong CALI , offset[BMA250_AXIS_X] = %d , offset[BMA250_AXIS_Y] = %d, offset[BMA250_AXIS_Z] = %d\n",
			obj->cali_sw[BMA250_AXIS_X],obj->cali_sw[BMA250_AXIS_Y],obj->cali_sw[BMA250_AXIS_Z]);

		}
	return 0;
}
//add by wangyufei@yulong.com modify for gsensor calibration end 20150415


/*----------------------------------------------------------------------------*/
int bma250_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct bma250_i2c_data *priv = (struct bma250_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[BMA250_BUFSIZE];
	
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
					sample_delay = BMA250_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = BMA250_BW_100HZ;
				}
				else
				{
					sample_delay = BMA250_BW_50HZ;
				}
				
				err = BMA250_SetBWRate(priv->client, sample_delay);
				if(err != BMA250_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
				#if defined(CONFIG_BMA250_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[BMA250_AXIS_X] = 0;
					priv->fir.sum[BMA250_AXIS_Y] = 0;
					priv->fir.sum[BMA250_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				#endif
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
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					err = BMA250_SetPowerMode( priv->client, !sensor_power);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				BMA250_ReadSensorData(priv->client, buff, BMA250_BUFSIZE);
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

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int bma250_open(struct inode *inode, struct file *file)
{
	file->private_data = bma250_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma250_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


//add by wangyufei@yulong.com add for gensor calibration begin 20150416
static int yulong_accel_Calibration(struct i2c_client *client, char *buf, int bufsize,int count,unsigned long cal_arg)
{
    int ret = 0;
    //char databuf[6];
    char i;
    //int data[3];
    //char tmp[1];
    int cali[3];
    char strbuf[BMA250_BUFSIZE];
    long xavg = 0, yavg = 0, zavg = 0;
    char tempbuf[NAND_FLASH_WR_RD_SIZE];
    //int ret = 0;
    struct acc_offset acc_cal_data;
    struct acc_offset *acc_cal_ptr = &acc_cal_data;

    struct bma250_i2c_data *obj = i2c_get_clientdata(client);

    BMA250_power(obj->hw, 1);
    msleep(50);
    if((ret = BMA250_SetPowerMode(obj->client, true)) > 0)
    {
        GSE_ERR("write power control fail wangyufei err is %d!!\n",ret);
        //return;
    }


    for (i = 0; i < count; i++)
    {
    if((ret = BMA250_ReadSensorData(client, strbuf, BMA250_BUFSIZE)) > 0)
        GSE_ERR("wangyufei I2C error: ret value=%d\n", ret);

        xavg += acc_data_b[BMA250_AXIS_X];
        yavg += acc_data_b[BMA250_AXIS_Y];
        zavg += acc_data_b[BMA250_AXIS_Z];
    }


    printk("(%d),xavg is %ld,Yavg is %ld,Zavg is %ld\n",__LINE__,xavg,yavg,zavg);  
    cali[BMA250_AXIS_X] = xavg/count;
    cali[BMA250_AXIS_Y] = yavg/count;
    cali[BMA250_AXIS_Z] = zavg/count;
    printk("(%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[BMA250_AXIS_X],cali[BMA250_AXIS_Y],cali[BMA250_AXIS_Z]);
    cali[BMA250_AXIS_X] = 0-cali[BMA250_AXIS_X];
    cali[BMA250_AXIS_Y] = 0-cali[BMA250_AXIS_Y];
    if(-9300 >= cali[BMA250_AXIS_Z])                    //wangyufei@yulong.com add for opposite gsensor calilbration begin at 20150601
        cali[BMA250_AXIS_Z] = 9807+cali[BMA250_AXIS_Z];
    else
        cali[BMA250_AXIS_Z] = 9807-cali[BMA250_AXIS_Z]; //wangyufei@yulong.com add for opposite gsensor calilbration begin at 20150601

    cali[BMA250_AXIS_X] = cali[BMA250_AXIS_X] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[BMA250_AXIS_Y] = cali[BMA250_AXIS_Y] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[BMA250_AXIS_Z] = cali[BMA250_AXIS_Z] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    printk("(%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[BMA250_AXIS_X],cali[BMA250_AXIS_Y],cali[BMA250_AXIS_Z]);

    //wangyufei@yulong.com modify for acc calibration begin at 20141205
    if(((CAL_FUZZ > cali[BMA250_AXIS_X])&&(-CAL_FUZZ < cali[BMA250_AXIS_X]))&&((CAL_FUZZ > cali[BMA250_AXIS_Y])&&(-CAL_FUZZ < cali[BMA250_AXIS_Y]))
        &&((CAL_FUZZ > cali[BMA250_AXIS_Z])&&(-CAL_FUZZ < cali[BMA250_AXIS_Z])))
    {
        ret = BMA250_WriteCalibration(client, cali);
        //GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!xavg =%d,yavg =%d,zavg =%d \n",xavg,yavg,zavg);
        GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!obj->reso->sensitivity = %d,cali[BMA250_AXIS_X] =%d,scali[BMA250_AXIS_Y] =%d,cali[BMA250_AXIS_Z] =%d \n",obj->reso->sensitivity,cali[BMA250_AXIS_X],cali[BMA250_AXIS_Y],cali[BMA250_AXIS_Z]);


        ((struct acc_offset *)acc_cal_ptr)->x = obj->cali_sw[BMA250_AXIS_X];
        ((struct acc_offset *)acc_cal_ptr)->y = obj->cali_sw[BMA250_AXIS_Y];
        ((struct acc_offset *)acc_cal_ptr)->z = obj->cali_sw[BMA250_AXIS_Z];
        ((struct acc_offset *)acc_cal_ptr)->key = ret ? 2 : 1;

        printk("(%d),obj->acc_cali_sw[BMA250_AXIS_X] is %d,obj->acc_cali_sw[BMA250_AXIS_Y] is %d,obj->acc_cali_sw[BMA250_AXIS_Z] is %d\n",
            __LINE__,obj->cali_sw[BMA250_AXIS_X],obj->cali_sw[BMA250_AXIS_Y],obj->cali_sw[BMA250_AXIS_Z]);      

        printk(KERN_INFO "%s write: x = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
        printk(KERN_INFO "%s write: y = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
        printk(KERN_INFO "%s write: z = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->z);
        printk(KERN_INFO "%s write: key = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->key);
        memset(tempbuf, 0, sizeof(tempbuf));

        tempbuf[0]=0x01;
        tempbuf[1] =  (obj->cali_sw[BMA250_AXIS_X] & 0xff00) >> 8;
        tempbuf[2] = obj->cali_sw[BMA250_AXIS_X] & 0x00ff;
        tempbuf[3] = (obj->cali_sw[BMA250_AXIS_Y] & 0xff00) >> 8;
        tempbuf[4] = obj->cali_sw[BMA250_AXIS_Y] & 0x00ff;
        tempbuf[5] = (obj->cali_sw[BMA250_AXIS_Z] & 0xff00) >> 8;
        tempbuf[6] = obj->cali_sw[BMA250_AXIS_Z] & 0x00ff;

        if(copy_to_user((struct acc_offset *)cal_arg, acc_cal_ptr, sizeof(acc_cal_data)))
        {
            printk("%s:  Calibrate copy_to_user failed!\n", __func__);          
        }
      //  printk("(%s) TEMPBUF is (%d),(%d),(%d),(%d),(%d),(%d),(%d)\n",
      //      __LINE__,tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6] );
      //  printk("(%s) TEMPBUF is (%x),(%x),(%x),(%x),(%x),(%x),(%x)\n",
      //      __LINE__,tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6] );

        printk(" wangyufei_write _acc_ps_cal_data  %s\n", __func__);
        if(write_acc_ps_cal_data_to_flash(16, tempbuf, NAND_FLASH_WR_RD_SIZE)<0)
            printk("Create ACC  calibration file error!!");
            else
            printk("Create ACC  calibration file Success!!");

        return 0;
    }
    else
    {
        printk("%s:  the calibration is out of offset!!!\n", __func__);
        return -1;
    }
    //wangyufei@yulong.com modify for acc calibration end at 20141205

}

//add by wangyufei@yulong.com add for gsensor calibration end 20150416

/*----------------------------------------------------------------------------*/
//static int bma250_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long bma250_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma250_i2c_data *obj = (struct bma250_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA250_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
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

			//add by wangyufei@yulong.com add for gsensor calibration begin 20150416
		case ACC_CALIBRATE:
			BMA250_power(obj->hw, 1);
			msleep(50);
			if((err = BMA250_SetPowerMode(obj->client, true)) > 0)
			{
				GSE_ERR("write power control fail wangyufei err is %ld!!\n",err);
		
			}
			GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
			err = BMA250_ResetCalibration(client);	//clear cali
			
			printk("wangyufei acc_calibrate in %s,%d\n",__func__,__LINE__);
            //wangyufei@yulong.com modify for acc calibration begin at 20141205
            err = yulong_accel_Calibration(client, strbuf, BMA250_BUFSIZE,20,arg);
            if(0 > err)
            {
                GSE_ERR("accel calibration wangyufei err is %ld!!\n",err);
                err = -EINVAL;
            }
            //wangyufei@yulong.com modify for gsensor calibration end at 20141205
		  break;

			
		//add by wangyufei end 20140630	
		case GSENSOR_IOCTL_INIT:
			bma250_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA250_ReadChipInfo(client, strbuf, BMA250_BUFSIZE);
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
			
			BMA250_ReadSensorData(client, strbuf, BMA250_BUFSIZE);
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
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA250_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			GSE_LOG("wangyufei fwq GSENSOR_IOCTL_SET_CALI!!\n");
			yulong_acc_ReadCalibration(client);//add by wangyufei@yulong.com for gsensor calibration at 20150416
			
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
				cali[BMA250_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA250_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA250_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = BMA250_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA250_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = BMA250_ReadCalibration(client, cali)) > 0)
			{
				break;
			}
			
			sensor_data.x = cali[BMA250_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA250_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA250_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
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


/*----------------------------------------------------------------------------*/
static struct file_operations bma250_fops = {
	//.owner = THIS_MODULE,
	.open = bma250_open,
	.release = bma250_release,
	.unlocked_ioctl = bma250_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma250_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma250_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef CONFIG_HAS_EARLYSUSPEND
#if 1
/*----------------------------------------------------------------------------*/
static int bma250_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);    
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
		if((err = BMA250_SetPowerMode(obj->client, false)) > 0)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}       
		BMA250_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma250_resume(struct i2c_client *client)
{
	struct bma250_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	BMA250_power(obj->hw, 1);
	if((err = bma250_init_client(client, 0)) > 0)
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma250_early_suspend(struct early_suspend *h) 
{
	struct bma250_i2c_data *obj = container_of(h, struct bma250_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	if(err = BMA250_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	BMA250_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void bma250_late_resume(struct early_suspend *h)
{
	struct bma250_i2c_data *obj = container_of(h, struct bma250_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
//wangyufei@yulong.com modify for gsensor resume optimize begin at 20150416
	BMA250_power(obj->hw, 1);
	msleep(50);
	if(err = BMA250_SetPowerMode(obj->client, true))
	{
		GSE_ERR("write power control fail wangyufei err is %d!!\n",err);
		//return;
	}	
	GSE_ERR("late_resume_ wangyufei err is %d!!\n",err);
//wangyufei@yulong.com modify for gsensor resume optimize end at 20150416
/*	BMA250_power(obj->hw, 1);
	if(err = bma250_init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
*/
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
//static int bma250_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
//{    
//	strcpy(info->type, BMA250_DEV_NAME);
//	return 0;
//}

extern int get_device_info(char* buf); //add by wangyufei@yulong.com for device list , 2014.06.26
/*
static struct sensors_classdev sensors_accelerometer_cdev = {
         .name = "accelerometer",
         .vendor = "kxtj2_1009",
         .version = 1,
         .handle = SENSORS_ACCELERATION_HANDLE,
         .type = SENSOR_TYPE_ACCELEROMETER,
         .max_range = "100000",
         .resolution = "1.0",
         .sensor_power = "0.1",
         .min_delay = 20000, // in microseconds
         .fifo_reserved_event_count = 0,
         .fifo_max_event_count = 0,
         .sensors_enable = NULL,
         .sensors_poll_delay = NULL,
};
*/
/*----------------------------------------------------------------------------*/
/*
  * huyue added on 2016/02/22
  */
static int bma250_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true

    return 0;
}

/*
 * huyue added on 2016/02/22
 */
static int bma250_enable_nodata(int en)
{
    int value = en;
    int err = 0;
    static uint8_t enable_flag = 0;  //shihaobin add read calibration data 20150330
    struct bma250_i2c_data *priv = obj_i2c_data;

    if(priv == NULL)
    {
        GSE_ERR("obj_i2c_data is NULL!\n");
        return -1;
    }

    //shihaobin add for read calibration data when first enable sensor begin 20150330
    if (0 == enable_flag)
    {
        enable_flag = 1;
        yulong_acc_ReadCalibration(bma250_i2c_client);
    }
    //shihaobin add for read calibration data when first enable sensor end 20150330


    GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);

	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		err = BMA250_SetPowerMode( priv->client, !sensor_power);
	}

    GSE_LOG("%s OK!\n",__FUNCTION__);
    return err;
}

/*
 * huyue added on 2016/02/22
 */
static int bma250_set_delay(u64 ns)
{
    int value =0;
    int err = 0;
    int sample_delay;
    struct bma250_i2c_data *priv = obj_i2c_data;

    value = (int)ns/1000/1000;

    if(priv == NULL)
    {
        GSE_ERR("obj_i2c_data is NULL!\n");
        return -1;
    }

	if(value <= 5)
	{
		sample_delay = BMA250_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = BMA250_BW_100HZ;
	}
	else
	{
		sample_delay = BMA250_BW_50HZ;
	}

	err = BMA250_SetBWRate(priv->client, sample_delay);
	if(err != BMA250_SUCCESS )
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{
	#if defined(CONFIG_BMA250_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[BMA250_AXIS_X] = 0;
		priv->fir.sum[BMA250_AXIS_Y] = 0;
		priv->fir.sum[BMA250_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	#endif
	}

    GSE_LOG("%s (%d), chip only use 1024HZ \n",__FUNCTION__, value);

    return 0;
}


/*
 * huyue added on 2016/02/22
 */
static int bma250_get_data(int *x , int *y, int *z, int *status)
{
	char buff[BMA250_BUFSIZE];
	int ret;

	BMA250_ReadSensorData(obj_i2c_data->client, buff, BMA250_BUFSIZE);
	ret = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

/*start to modify for chip info by qinxinjun@yulong.com in 20160504*/
static struct sensors_classdev sensors_accelerometer_cdev = {
         .name = "accelerometer",
         .vendor = "BMA250",
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

static int bma250_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma250_i2c_data *obj;
	struct hwmsen_object sobj;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
    int retry = 0;

	int err = 0;
	char tembuff[50]={0}; //add by wangyufei@yulong.com for device list , 2014.06.26

	GSE_FUN();
    /*start to modify for chip info by qinxinjun@yulong.com in 20160504*/
    sensors_classdev_register(&client->dev,&sensors_accelerometer_cdev);
    /*end to modify for chip info by qinxinjun@yulong.com in 20160504*/

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct bma250_i2c_data));

	obj->hw = hw;

    printk("BMA250E direction is %d\n", obj->hw->direction);

	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)) > 0)
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

#ifdef CONFIG_BMA250_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	bma250_i2c_client = new_client;	

	for (retry = 0; retry < 2; retry++)
    {
        err = bma250_init_client(new_client, 1);
        if (0 != err)
        {
            GSE_ERR("bma250_device init cilent fail time: %d\n", retry);
            continue;
        }
    }
    if (0 != err)
        goto exit_init_failed;

	if((err = misc_register(&bma250_device)) > 0)
	{
		GSE_ERR("bma250_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/* add by mayulong 20130925 begin */
	if((err = bma250_create_attr(&(bma250_init_info.platform_diver_addr->driver))))
/* add by mayulong 20130925 end */
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
  //add by wangyufei@yulong.com for device list , 2014.06.6
  sprintf(tembuff,"ACC: BAM250 BOSCH \n");
  get_device_info(tembuff);

  //end by wangyufei@yulong.com for device list at 20140606

	ctl.open_report_data = bma250_open_report_data;
	ctl.enable_nodata = bma250_enable_nodata;
	ctl.set_delay  = bma250_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;
	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}
	data.get_data = bma250_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err= %d\n", err);
		goto exit_kfree;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = bma250_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)) > 0)
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
	
	if((err = hwmsen_attach(ID_GRAVITY, &sobj)) > 0)
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = bma250_early_suspend,
	obj->early_drv.resume   = bma250_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
#endif

// begin baibo @yulong.com 20150915 for gsensor compat
//#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
        bma250_init_flag = 0;
#endif
// end baibo @yulong.com 20150915 for gsensor compatt
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&bma250_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
// begin baibo @yulong.com 20150915 for gsensor compat
//#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
        bma250_init_flag = -1;
#endif
// end baibo @yulong.com 20150915 for gsensor compat
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma250_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

        /* modify by mayulong 20130925 begin */
        //#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
        #if 1
	if((err = bma250_delete_attr(&(bma250_init_info.platform_diver_addr->driver))))
        #else
        if(err = bma250_delete_attr(&bma250_gsensor_driver.driver))
        #endif
        /* modify by mayulong 20130925 end */
	{
		GSE_ERR("bma150_delete_attr fail: %d\n", err);
	}
	
	if((err = misc_deregister(&bma250_device)) > 0)
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	//if((err = hwmsen_detach(ID_ACCELEROMETER)) > 0)

	bma250_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
/* modify by mayulong 20130925 begin */
//#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
static int bma250_remove(void)
{
    //struct acc_hw *hw = bma250_get_cust_acc_hw();

    GSE_FUN();    
    BMA250_power(hw, 0);    
    i2c_del_driver(&bma250_i2c_driver);
    return 0;
}
static int bma250_local_init(void) 
{
	//struct acc_hw *hw = bma250_get_cust_acc_hw();
	GSE_FUN();

	BMA250_power(hw, 1);
	//bma250_force[0] = hw->i2c_num;
	if(i2c_add_driver(&bma250_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
        if(-1 == bma250_init_flag)
        {
                return -1;
        }
	return 0;
}
#else
static int bma250_probe(struct platform_device *pdev) 
{
	//struct acc_hw *hw = bma250_get_cust_acc_hw();
	GSE_FUN();

	BMA250_power(hw, 1);
	//bma250_force[0] = hw->i2c_num;
	if(i2c_add_driver(&bma250_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma250_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = bma250_get_cust_acc_hw();

    GSE_FUN();    
    BMA250_power(hw, 0);    
    i2c_del_driver(&bma250_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
#if 1
#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor", },
	{},
};
#endif

static struct platform_driver bma250_gsensor_driver = {
	.probe      = bma250_probe,
	.remove     = bma250_remove,    
	.driver     = 
	{
		.name  = "gsensor",
		.owner  = THIS_MODULE,
        #ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
		#endif
	}
};
#else

static struct platform_driver bma250_gsensor_driver = {
	.probe      = bma250_probe,
	.remove     = bma250_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};
#endif
#endif
/* modify by mayulong 20130925 end */
/*----------------------------------------------------------------------------*/
static int __init bma250_init(void)
{
	const char *name = "mediatek,bma250";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&bma250_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit bma250_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
//module_init(bma250_init);
fs_initcall(bma250_init);
module_exit(bma250_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA250 I2C driver");
MODULE_AUTHOR("Xiaoli.li@mediatek.com");
