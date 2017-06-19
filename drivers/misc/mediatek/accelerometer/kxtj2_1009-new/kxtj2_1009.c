/* KXTJ2_1009 motion sensor driver
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
 * modify by wangyufei at 20141016
 * port to 6.0 by huyue at 20160525
 */

#include "cust_acc.h"
#include "accel.h"
#include "kxtj2_1009.h"
//#include <linux/sensors.h>/*add for chip info by qinxinjun@yulong.com in 20160504*/
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_KXTJ2_1009 150
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_KXTJ2_1009_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define KXTJ2_1009_AXIS_X          0
#define KXTJ2_1009_AXIS_Y          1
#define KXTJ2_1009_AXIS_Z          2
#define KXTJ2_1009_AXES_NUM        3
#define KXTJ2_1009_DATA_LEN        6
#define KXTJ2_1009_DEV_NAME        "KXTJ2_1009"
/*----------------------------------------------------------------------------*/

// add by wangyufei 20140522 begin 
#define NAND_FLASH_WR_RD_SIZE 10
//add by wangyufei 20140520 end 
static const struct i2c_device_id kxtj2_1009_i2c_id[] = {{KXTJ2_1009_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_kxtj2_1009={ I2C_BOARD_INFO(KXTJ2_1009_DEV_NAME, (KXTJ2_1009_I2C_SLAVE_ADDR>>1))};
/*the adapter id will be available in customization*/
//static unsigned short kxtj2_1009_force[] = {0x00, KXTJ2_1009_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const kxtj2_1009_forces[] = { kxtj2_1009_force, NULL };
//static struct i2c_client_address_data kxtj2_1009_addr_data = { .forces = kxtj2_1009_forces,};

typedef struct {
	int x;
	int y;
	int z;
} SENSOR_DATA;

static struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
        {.compatible = "mediatek,gsensor_1"},
        {},
};
#endif


/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int kxtj2_1009_i2c_remove(struct i2c_client *client);
//static int kxtj2_1009_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int kxtj2_1009_suspend(struct i2c_client *client, pm_message_t msg);
static int kxtj2_1009_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
//add by wangyufei 20140522 begin

#define CAL_FUZZ 380                //wangyufei@yulong.com modify for acc calibration at 20141205
#ifdef CAL_MODE
s16 acc_data[3];
//static int match_sensor_flag = 0;
extern int yl_params_kernel_write(char *buf, size_t count, int ver);    //wangyufei@yulong.com add yl_params at 20150521
extern int yl_params_kernel_read(char *buf, size_t count, int ver);     //wangyufei@yulong.com add yl_params at 20150521
extern int read_acc_ps_cal_data_from_flash(unsigned int offset, char* output, int counts);
extern int write_acc_ps_cal_data_to_flash(unsigned int offset, char* input, int counts);
#endif
// add by wangyufei 20140522 end
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
    s16 raw[C_MAX_FIR_LENGTH][KXTJ2_1009_AXES_NUM];
    int sum[KXTJ2_1009_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct kxtj2_1009_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[KXTJ2_1009_AXES_NUM+1];

    /*data*/
    s8                      offset[KXTJ2_1009_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[KXTJ2_1009_AXES_NUM+1];

#if defined(CONFIG_KXTJ2_1009_LOWPASS)
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
static struct i2c_driver kxtj2_1009_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = KXTJ2_1009_DEV_NAME,
        #ifdef CONFIG_OF
		.of_match_table = accel_of_match,
		#endif
    },
	.probe      		= kxtj2_1009_i2c_probe,
	.remove    			= kxtj2_1009_i2c_remove,
//	.detect				= kxtj2_1009_i2c_detect,
//#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = kxtj2_1009_suspend,
    .resume             = kxtj2_1009_resume,
//#endif
	.id_table = kxtj2_1009_i2c_id,
//	.address_data = &kxtj2_1009_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *kxtj2_1009_i2c_client = NULL;
//static struct platform_driver kxtj2_1009_gsensor_driver;
static struct kxtj2_1009_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[8]= {0}; 
static DEFINE_MUTEX(kxtj2_1009_mutex);
static bool enable_status = false;

/* add by mayulong 20130925 begin */
//#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
static int kxtj2_1009_init_flag = -1;   //wangyufei@yulong.com modify for gsensor compat at 20141031
static int  kxtj2_1009_local_init(void);
static int  kxtj2_1009_remove(void);


static struct acc_init_info kxtj2_1009_init_info = {
		.name = "kxtj2_1009",
		.init = kxtj2_1009_local_init,
		.uninit = kxtj2_1009_remove,
};
#endif
/* add by mayulong 20130925 end */
/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution kxtj2_1009_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 0, 9}, 1024}, // dataformat +/-2g  in 12-bit resolution;  { 3, 9} = 3.9 = (2*2*1000)/(2^12);  256 = (2^12)/(2*2)          
};
/*----------------------------------------------------------------------------*/
static struct data_resolution kxtj2_1009_offset_resolution = {{15, 6}, 64};
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetPowerMode(struct i2c_client *client, bool enable);
/*--------------------KXTJ2_1009 power control function----------------------------------*/
static void KXTJ2_1009_power(struct acc_hw *hw, unsigned int on) 
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
			if(!hwPowerOn(hw->power_id, hw->power_vol, "KXTJ2_1009"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "KXTJ2_1009"))
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
static int KXTJ2_1009_SetDataResolution(struct kxtj2_1009_i2c_data *obj)
{
	int err;
	u8  databuf[2];
	bool cur_sensor_power = sensor_power;

	KXTJ2_1009_SetPowerMode(obj->client, false);

	if(hwmsen_read_block(obj->client, KXTJ2_1009_REG_DATA_RESOLUTION, databuf, 0x01))
	{
		printk("kxtj2_1009 read Dataformat failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= ~KXTJ2_1009_RANGE_DATA_RESOLUTION_MASK;
	databuf[0] |= KXTJ2_1009_RANGE_DATA_RESOLUTION_MASK;//12bit
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_DATA_RESOLUTION;


	err = i2c_master_send(obj->client, databuf, 0x2);

	if(err <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	KXTJ2_1009_SetPowerMode(obj->client, cur_sensor_power/*true*/);

	//kxtj2_1009_data_resolution[0] has been set when initialize: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/   
	obj->reso = &kxtj2_1009_data_resolution[0];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadData(struct i2c_client *client, s16 data[KXTJ2_1009_AXES_NUM])
{
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	struct kxtj2_1009_i2c_data *priv = i2c_get_clientdata(client);        
#endif
	u8 addr = KXTJ2_1009_REG_DATAX0;
	u8 buf[KXTJ2_1009_DATA_LEN] = {0};
	int err = 0;
	int i;



	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = hwmsen_read_block(client, addr, buf, 0x06)))
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[KXTJ2_1009_AXIS_X] = (s16)((buf[KXTJ2_1009_AXIS_X*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_X*2+1] << 4));
		data[KXTJ2_1009_AXIS_Y] = (s16)((buf[KXTJ2_1009_AXIS_Y*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_Y*2+1] << 4));
		data[KXTJ2_1009_AXIS_Z] = (s16)((buf[KXTJ2_1009_AXIS_Z*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_Z*2+1] << 4));

		for(i=0;i<3;i++)				
		{								//because the data is store in binary complement number formation in computer system
			if ( data[i] == 0x0800 )	//so we want to calculate actual number here
				data[i]= -2048;			//10bit resolution, 512= 2^(12-1)
			else if ( data[i] & 0x0800 )//transfor format
			{							//printk("data 0 step %x \n",data[i]);
				data[i] -= 0x1; 		//printk("data 1 step %x \n",data[i]);
				data[i] = ~data[i]; 	//printk("data 2 step %x \n",data[i]);
				data[i] &= 0x07ff;		//printk("data 3 step %x \n\n",data[i]);
				data[i] = -data[i]; 	
			}
		}	


		//if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z],
		                               data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z]);
		}
#ifdef CONFIG_KXTJ2_1009_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_X] = data[KXTJ2_1009_AXIS_X];
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Y] = data[KXTJ2_1009_AXIS_Y];
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Z] = data[KXTJ2_1009_AXIS_Z];
					priv->fir.sum[KXTJ2_1009_AXIS_X] += data[KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] += data[KXTJ2_1009IK_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] += data[KXTJ2_1009_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_X], priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Y], priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Z],
							priv->fir.sum[KXTJ2_1009_AXIS_X], priv->fir.sum[KXTJ2_1009_AXIS_Y], priv->fir.sum[KXTJ2_1009_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[KXTJ2_1009_AXIS_X] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_Z];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_X] = data[KXTJ2_1009_AXIS_X];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_Y] = data[KXTJ2_1009_AXIS_Y];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_Z] = data[KXTJ2_1009_AXIS_Z];
					priv->fir.sum[KXTJ2_1009_AXIS_X] += data[KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] += data[KXTJ2_1009_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] += data[KXTJ2_1009_AXIS_Z];
					priv->fir.idx++;
					data[KXTJ2_1009_AXIS_X] = priv->fir.sum[KXTJ2_1009_AXIS_X]/firlen;
					data[KXTJ2_1009_AXIS_Y] = priv->fir.sum[KXTJ2_1009_AXIS_Y]/firlen;
					data[KXTJ2_1009_AXIS_Z] = priv->fir.sum[KXTJ2_1009_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][KXTJ2_1009_AXIS_X], priv->fir.raw[idx][KXTJ2_1009_AXIS_Y], priv->fir.raw[idx][KXTJ2_1009_AXIS_Z],
						priv->fir.sum[KXTJ2_1009_AXIS_X], priv->fir.sum[KXTJ2_1009_AXIS_Y], priv->fir.sum[KXTJ2_1009_AXIS_Z],
						data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadOffset(struct i2c_client *client, s8 ofs[KXTJ2_1009_AXES_NUM])
{    
	int err = 0;

	ofs[1]=ofs[2]=ofs[0]=0x00;

	printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ResetCalibration(struct i2c_client *client)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	int err = 0;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadCalibration(struct i2c_client *client, int dat[KXTJ2_1009_AXES_NUM])
{
    struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
#ifndef SW_CALIBRATION
    int err;
#endif
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = KXTJ2_1009_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X]);
    dat[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y]);
    dat[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z]);                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadCalibrationEx(struct i2c_client *client, int act[KXTJ2_1009_AXES_NUM], int raw[KXTJ2_1009_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
#ifndef SW_CALIBRATION
	int err;
#endif
	int mul;
 

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if(err = KXTJ2_1009_ReadOffset(client, obj->offset))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
	#endif
	
	raw[KXTJ2_1009_AXIS_X] = obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X];
	raw[KXTJ2_1009_AXIS_Y] = obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y];
	raw[KXTJ2_1009_AXIS_Z] = obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z];

	act[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*raw[KXTJ2_1009_AXIS_X];
	act[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*raw[KXTJ2_1009_AXIS_Y];
	act[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*raw[KXTJ2_1009_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_WriteCalibration(struct i2c_client *client, int dat[KXTJ2_1009_AXES_NUM])
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[KXTJ2_1009_AXES_NUM], raw[KXTJ2_1009_AXES_NUM];
#ifndef SW_CALIBRATION
	int lsb = kxtj2_1009_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif

	if((err = KXTJ2_1009_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[KXTJ2_1009_AXIS_X], raw[KXTJ2_1009_AXIS_Y], raw[KXTJ2_1009_AXIS_Z],
		obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
		obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[KXTJ2_1009_AXIS_X] += dat[KXTJ2_1009_AXIS_X];
	cali[KXTJ2_1009_AXIS_Y] += dat[KXTJ2_1009_AXIS_Y];
	cali[KXTJ2_1009_AXIS_Z] += dat[KXTJ2_1009_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[KXTJ2_1009_AXIS_X], dat[KXTJ2_1009_AXIS_Y], dat[KXTJ2_1009_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[KXTJ2_1009_AXIS_X] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]]);
	obj->cali_sw[KXTJ2_1009_AXIS_Y] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]]);
	obj->cali_sw[KXTJ2_1009_AXIS_Z] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]]);	
	printk("(%d),obj->cali_sw[KXTJ2_1009_AXIS_X] is %d,obj->cali_sw[KXTJ2_1009_AXIS_Y] is %d,obj->cali_sw[KXTJ2_1009_AXIS_Z] is %d\n",
		__LINE__,obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);	
#else
	obj->offset[KXTJ2_1009_AXIS_X] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]])/(divisor));
	obj->offset[KXTJ2_1009_AXIS_Y] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]])/(divisor));
	obj->offset[KXTJ2_1009_AXIS_Z] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[KXTJ2_1009_AXIS_X] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]])%(divisor);
	obj->cali_sw[KXTJ2_1009_AXIS_Y] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]])%(divisor);
	obj->cali_sw[KXTJ2_1009_AXIS_Z] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[KXTJ2_1009_AXIS_X]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_X], 
		obj->offset[KXTJ2_1009_AXIS_Y]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_Y], 
		obj->offset[KXTJ2_1009_AXIS_Z]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_Z], 
		obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
		obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

	printk("(%d),obj->cali_sw[KXTJ2_1009_AXIS_X] is %d,obj->cali_sw[KXTJ2_1009_AXIS_Y] is %d,obj->cali_sw[KXTJ2_1009_AXIS_Z] is %d\n",
		__LINE__,obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);	

	printk("(%d),obj->offset[KXTJ2_1009_AXIS_X] is %d,obj->offset[KXTJ2_1009_AXIS_Y] is %d,obj->offset[[KXTJ2_1009_AXIS_Z] is %d\n",
		__LINE__,obj->offset[KXTJ2_1009_AXIS_X],obj->offset[KXTJ2_1009_AXIS_Y],obj->offset[KXTJ2_1009_AXIS_Z]);	

	if(err = hwmsen_write_block(obj->client, KXTJ2_1009_REG_OFSX, obj->offset, KXTJ2_1009_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = KXTJ2_1009_REG_DEVID;   

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_KXTJ2_1009_CheckDeviceID;
	}
	
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_KXTJ2_1009_CheckDeviceID;
	}
	

	if(false)
	{
		printk("KXTJ2_1009_CheckDeviceID 0x%x failt!\n ", databuf[0]);
		return KXTJ2_1009_ERR_IDENTIFICATION;
	}
	else
	{
		printk("KXTJ2_1009_CheckDeviceID 0x%x pass!\n ", databuf[0]);
	}
	
	exit_KXTJ2_1009_CheckDeviceID:
	if (res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	
	return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = KXTJ2_1009_REG_POWER_CTL;
	
	
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return KXTJ2_1009_SUCCESS;
	}

	if(hwmsen_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXTJ2_1009_ERR_I2C;
	}

	
	if(enable == true)
	{
		databuf[0] |= KXTJ2_1009_MEASURE_MODE;
	}
	else
	{
		databuf[0] &= ~KXTJ2_1009_MEASURE_MODE;
	}
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_POWER_CTL;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}


	GSE_LOG("KXTJ2_1009_SetPowerMode %d!\n ",enable);


	sensor_power = enable;

	mdelay(5);
	
	return KXTJ2_1009_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;
	bool cur_sensor_power = sensor_power;

	memset(databuf, 0, sizeof(u8)*10);  

	KXTJ2_1009_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXTJ2_1009_REG_DATA_FORMAT, databuf, 0x01))
	{
		printk("kxtj2_1009 read Dataformat failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= ~KXTJ2_1009_RANGE_MASK;
	databuf[0] |= dataformat;
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_DATA_FORMAT;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	KXTJ2_1009_SetPowerMode(client, cur_sensor_power/*true*/);
	
	printk("KXTJ2_1009_SetDataFormat OK! \n");
	

	return KXTJ2_1009_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;
	bool cur_sensor_power = sensor_power;

	memset(databuf, 0, sizeof(u8)*10);    

	
	KXTJ2_1009_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXTJ2_1009_REG_BW_RATE, databuf, 0x01))
	{
		printk("kxtj2_1009 read rate failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= 0xf0;
	databuf[0] |= bwrate;
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_BW_RATE;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	
	KXTJ2_1009_SetPowerMode(client, cur_sensor_power/*true*/);
	printk("KXTJ2_1009_SetBWRate OK! \n");
	
	return KXTJ2_1009_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = KXTJ2_1009_REG_INT_ENABLE;    
	databuf[1] = 0x00;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	
	return KXTJ2_1009_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_init_client(struct i2c_client *client, int reset_cali)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);   //wangyufei@yulong.com modify for gsensor compat begin at 20141031
	int res = 0;
	int i = 0;
	printk("wangyufei in init_client 11!\n");
	for(i=0;i<3;i++)
	{
			res = KXTJ2_1009_CheckDeviceID(client); 
			printk("wangyufei in init checkdeviceId res is %d!\n",res);
			if(0 == res)
				break;
	}
	if(res != KXTJ2_1009_SUCCESS)
	{
		printk("wangyufei in checkdeviceId res is %d!\n",res);
		return res;
	}														//wangyufei@yulong.com modify for gsensor compat end at 20141031

	res = KXTJ2_1009_SetPowerMode(client, enable_status/*false*/);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	

	res = KXTJ2_1009_SetBWRate(client, KXTJ2_1009_BW_100HZ);
	if(res != KXTJ2_1009_SUCCESS ) //0x2C->BW=100Hz
	{
		return res;
	}

	res = KXTJ2_1009_SetDataFormat(client, KXTJ2_1009_RANGE_2G);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


	res = KXTJ2_1009_SetIntEnable(client, 0x00);        
	if(res != KXTJ2_1009_SUCCESS)//0x2E->0x80
	{
		return res;
	}

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = KXTJ2_1009_ResetCalibration(client);
		if(res != KXTJ2_1009_SUCCESS)
		{
			return res;
		}
	}
	printk("kxtj2_1009_init_client OK!\n");
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
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

	sprintf(buf, "KXTJ2_1009 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[KXTJ2_1009_AXES_NUM];
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

	if (atomic_read(&obj->suspend))
	{
		return 0;
	}
	/*if(sensor_power == FALSE)
	{
		res = KXTJ2_1009_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on kxtj2_1009 error %d!\n", res);
		}
	}*/

	res = KXTJ2_1009_ReadData(client, obj->data);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[KXTJ2_1009_AXIS_X],obj->data[KXTJ2_1009_AXIS_Y],obj->data[KXTJ2_1009_AXIS_Z]);
		obj->data[KXTJ2_1009_AXIS_X] += obj->cali_sw[KXTJ2_1009_AXIS_X];
		obj->data[KXTJ2_1009_AXIS_Y] += obj->cali_sw[KXTJ2_1009_AXIS_Y];
		obj->data[KXTJ2_1009_AXIS_Z] += obj->cali_sw[KXTJ2_1009_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*obj->data[KXTJ2_1009_AXIS_X];
		acc[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*obj->data[KXTJ2_1009_AXIS_Y];
		acc[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*obj->data[KXTJ2_1009_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[KXTJ2_1009_AXIS_X],obj->cvt.sign[KXTJ2_1009_AXIS_Y],obj->cvt.sign[KXTJ2_1009_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[KXTJ2_1009_AXIS_X], acc[KXTJ2_1009_AXIS_Y], acc[KXTJ2_1009_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[KXTJ2_1009_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[KXTJ2_1009_AXIS_X] = acc[KXTJ2_1009_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXTJ2_1009_AXIS_Y] = acc[KXTJ2_1009_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXTJ2_1009_AXIS_Z] = acc[KXTJ2_1009_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		
		//add by wangyufei@yulong.com for acc calibration begin at 20141016
		acc_data[KXTJ2_1009_AXIS_X] = acc[KXTJ2_1009_AXIS_X];
		acc_data[KXTJ2_1009_AXIS_Y] = acc[KXTJ2_1009_AXIS_Y];
		acc_data[KXTJ2_1009_AXIS_Z] = acc[KXTJ2_1009_AXIS_Z];
	//	printk("wangyufei acc_data=%x, y=%x, z=%x",acc[KXTJ2_1009_AXIS_X],acc[KXTJ2_1009_AXIS_Y],acc[KXTJ2_1009_AXIS_Z]);
		//add by wangyufei@yulong.com for acc calibration end at 20141016	

		sprintf(buf, "%04x %04x %04x", acc[KXTJ2_1009_AXIS_X], acc[KXTJ2_1009_AXIS_Y], acc[KXTJ2_1009_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadRawData(struct i2c_client *client, char *buf)
{
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	res = KXTJ2_1009_ReadData(client, obj->data);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "KXTJ2_1009_ReadRawData %04x %04x %04x", obj->data[KXTJ2_1009_AXIS_X], 
			obj->data[KXTJ2_1009_AXIS_Y], obj->data[KXTJ2_1009_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8  data,result;
	
	res = hwmsen_read_byte(client, KXTJ2_1009_REG_CTL_REG3, &data);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
//enable selftest bit
	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  KXTJ2_1009_SELF_TEST|data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 1
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	printk("step1: result = %x",result);
	if(result != 0xaa)
		return -EINVAL;

//step 2
	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  KXTJ2_1009_SELF_TEST|data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 3
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	printk("step3: result = %x",result);
	if(result != 0xAA)
		return -EINVAL;
		
//step 4
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	printk("step4: result = %x",result);
	if(result != 0x55)
		return -EINVAL;
	else
		return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#if 0
static int KXTJ2_1009_JudgeTestResult(struct i2c_client *client, s32 prv[KXTJ2_1009_AXES_NUM], s32 nxt[KXTJ2_1009_AXES_NUM])
{
    int res=0;
	u8 test_result=0;
    if((res = hwmsen_read_byte(client, 0x0c, &test_result)))
        return res;

	printk("test_result = %x \n",test_result);
    if ( test_result != 0xaa ) 
	{
        GSE_ERR("KXTJ2_1009_JudgeTestResult failt\n");
        res = -EINVAL;
    }
    return res;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	char strbuf[KXTJ2_1009_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	KXTJ2_1009_ReadChipInfo(client, strbuf, KXTJ2_1009_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxtj2_1009_i2c_client;
		char strbuf[KXTJ2_1009_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		kxtj2_1009_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif



/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	char strbuf[KXTJ2_1009_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
	//KXTJ2_1009_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxtj2_1009_i2c_client;
		char strbuf[KXTJ2_1009_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
		KXTJ2_1009_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj;
	int err,err2, len = 0, mul;
	int tmp[KXTJ2_1009_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);


	err = KXTJ2_1009_ReadOffset(client, obj->offset);
	err2 = KXTJ2_1009_ReadCalibration(client, tmp);
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
		mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
			obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X],
			obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y],
			obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z],
			tmp[KXTJ2_1009_AXIS_X], tmp[KXTJ2_1009_AXIS_Y], tmp[KXTJ2_1009_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	int err, x, y, z;
	int dat[KXTJ2_1009_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = KXTJ2_1009_ResetCalibration(client))>0)
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[KXTJ2_1009_AXIS_X] = x;
		dat[KXTJ2_1009_AXIS_Y] = y;
		dat[KXTJ2_1009_AXIS_Z] = z;
		if((err = KXTJ2_1009_WriteCalibration(client, dat))>0)
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
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	//struct kxtj2_1009_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	//obj = i2c_get_clientdata(client);
	
	snprintf(buf, 8, "%s\n", selftestRes);
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{   /*write anything to this register will trigger the process*/
	struct item{
	s16 raw[KXTJ2_1009_AXES_NUM];
	};
	
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	int res, num;
	struct item *prv = NULL, *nxt = NULL;
	//s32 avg_prv[KXTJ2_1009_AXES_NUM] = {0, 0, 0};
	//s32 avg_nxt[KXTJ2_1009_AXES_NUM] = {0, 0, 0};
	u8 data;


	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}


	GSE_LOG("NORMAL:\n");
	KXTJ2_1009_SetPowerMode(client,true); 

	/*initial setting for self test*/
	if(!KXTJ2_1009_InitSelfTest(client))
	{
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes,"y");
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");		
		strcpy(selftestRes,"n");
	}

	res = hwmsen_read_byte(client, KXTJ2_1009_REG_CTL_REG3, &data);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}

	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  ~KXTJ2_1009_SELF_TEST&data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
	
	exit:
	/*restore the setting*/    
	kxtj2_1009_init_client(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
}
/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	int tmp;

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}
	
	
	if(1 == sscanf(buf, "%d", &tmp))
	{        
		if(atomic_read(&obj->selftest) && !tmp)
		{
			/*enable -> disable*/
			kxtj2_1009_init_client(obj->client, 0);
		}
		else if(!atomic_read(&obj->selftest) && tmp)
		{
			/*disable -> enable*/
			KXTJ2_1009_InitSelfTest(obj->client);            
		}
		
		GSE_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp); 
	}
	else
	{ 
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (unsigned int)count);   
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][KXTJ2_1009_AXIS_X], obj->fir.raw[idx][KXTJ2_1009_AXIS_Y], obj->fir.raw[idx][KXTJ2_1009_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[KXTJ2_1009_AXIS_X], obj->fir.sum[KXTJ2_1009_AXIS_Y], obj->fir.sum[KXTJ2_1009_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[KXTJ2_1009_AXIS_X]/len, obj->fir.sum[KXTJ2_1009_AXIS_Y]/len, obj->fir.sum[KXTJ2_1009_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
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
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
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
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
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
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (unsigned int)count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
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
	u8 databuf[2];    
	u8 addr = KXTJ2_1009_REG_POWER_CTL;
	if(hwmsen_read_block(kxtj2_1009_i2c_client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXTJ2_1009_ERR_I2C;
	}
    
	if(sensor_power)
		printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_self_value,  store_self_value);
static DRIVER_ATTR(self,   S_IWUSR | S_IRUGO, show_selftest_value,      store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);


/*----------------------------------------------------------------------------*/
static u8 i2c_dev_reg =0 ;

static ssize_t show_register(struct device_driver *pdri, char *buf)
{
	//int input_value;
		
	printk("i2c_dev_reg is 0x%2x \n", i2c_dev_reg);

	return 0;
}

static ssize_t store_register(struct device_driver *ddri, const char *buf, size_t count)
{
	//unsigned long input_value;

	i2c_dev_reg = simple_strtoul(buf, NULL, 16);
	printk("set i2c_dev_reg = 0x%2x \n", i2c_dev_reg);

	return 0;
}
static ssize_t store_register_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	u8 databuf[2];  
	unsigned long input_value;
	int res;
	
	memset(databuf, 0, sizeof(u8)*2);    

	input_value = simple_strtoul(buf, NULL, 16);
	printk("input_value = %ld \n", input_value);

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}

	databuf[0] = i2c_dev_reg;
	databuf[1] = input_value;
	printk("databuf[0]=0x%2x  databuf[1]=0x%2x \n", databuf[0],databuf[1]);

	res = i2c_master_send(obj->client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	return 0;
	
}

static ssize_t show_register_value(struct device_driver *ddri, char *buf)
{
		struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
		u8 databuf[1];	
		
		memset(databuf, 0, sizeof(u8)*1);	 
	
		if(NULL == obj)
		{
			GSE_ERR("i2c data obj is null!!\n");
			return 0;
		}
		
		if(hwmsen_read_block(obj->client, i2c_dev_reg, databuf, 0x01))
		{
			GSE_ERR("read power ctl register err!\n");
			return KXTJ2_1009_ERR_I2C;
		}

		printk("i2c_dev_reg=0x%2x  data=0x%2x \n", i2c_dev_reg,databuf[0]);
	
		return 0;
		
}


static DRIVER_ATTR(i2c,      S_IWUSR | S_IRUGO, show_register_value,         store_register_value);
static DRIVER_ATTR(register,      S_IWUSR | S_IRUGO, show_register,         store_register);


/*----------------------------------------------------------------------------*/
static struct driver_attribute *kxtj2_1009_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_self,         /*self test demo*/
	&driver_attr_selftest,     /*self control: 0: disable, 1: enable*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_register,
	&driver_attr_i2c,
};
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(kxtj2_1009_attr_list)/sizeof(kxtj2_1009_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, kxtj2_1009_attr_list[idx]))>0)
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", kxtj2_1009_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(kxtj2_1009_attr_list)/sizeof(kxtj2_1009_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, kxtj2_1009_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/
//add by wangyufei begin 20140630
static int yulong_acc_ReadCalibration(struct i2c_client *client)
{
	char tempbuf[NAND_FLASH_WR_RD_SIZE];
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);

	printk("WANGYUFEI IN YULONG ACC_READ_CALIBRATION!\n");
	
	if(read_acc_ps_cal_data_from_flash(16, tempbuf, NAND_FLASH_WR_RD_SIZE)<0)
		printk("Use Default CALI , acc_cali_sw[KXTJ2_1009_AXIS_X] = %d , acc_cali_sw[KXTJ2_1009_AXIS_Y] = %d, acc_cali_sw[KXTJ2_1009_AXIS_Z] = %d\n",
		obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);
	else{
	obj->cali_sw[KXTJ2_1009_AXIS_X] = (s16)((tempbuf[1]<<8)|tempbuf[2]);
	obj->cali_sw[KXTJ2_1009_AXIS_Y] = (s16)((tempbuf[3]<<8)|tempbuf[4]);
	obj->cali_sw[KXTJ2_1009_AXIS_Z] = (s16)((tempbuf[5]<<8)|tempbuf[6]);
			
		printk("Use yulong CALI ,tempbuf is %d,%d,%d,%d,%d,%d,%d ",
			tempbuf[0],tempbuf[1],tempbuf[2],tempbuf[3],tempbuf[4],tempbuf[5],tempbuf[6]);
	
		printk("Use yulong CALI , offset[KXTJ2_1009_AXIS_X] = %d , offset[KXTJ2_1009_AXIS_Y] = %d, offset[KXTJ2_1009_AXIS_Z] = %d\n",
			obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);

		}

	return 0;
}


//add by wangyufei end 20140630
int kxtj2_1009_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct kxtj2_1009_i2c_data *priv = (struct kxtj2_1009_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[KXTJ2_1009_BUFSIZE];
	
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
					sample_delay = KXTJ2_1009_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = KXTJ2_1009_BW_100HZ;
				}
				else
				{
					sample_delay = KXTJ2_1009_BW_50HZ;
				}

				mutex_lock(&kxtj2_1009_mutex);
				err = KXTJ2_1009_SetBWRate(priv->client, sample_delay);
				mutex_unlock(&kxtj2_1009_mutex);
				if(err != KXTJ2_1009_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
				#if defined(CONFIG_KXTJ2_1009_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_X] = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_Y] = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_Z] = 0;
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
				mutex_lock(&kxtj2_1009_mutex);
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					enable_status = sensor_power;
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					enable_status = !sensor_power;
					if (atomic_read(&priv->suspend) == 0)
					{
						err = KXTJ2_1009_SetPowerMode( priv->client, enable_status);
						GSE_LOG("Gsensor not in suspend KXTJ2_1009_SetPowerMode!, enable_status = %d\n",enable_status);
					}
					else
					{
						GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
					}
				}
				mutex_unlock(&kxtj2_1009_mutex);
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
				mutex_lock(&kxtj2_1009_mutex);
				KXTJ2_1009_ReadSensorData(priv->client, buff, KXTJ2_1009_BUFSIZE);
				mutex_unlock(&kxtj2_1009_mutex);
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
static int kxtj2_1009_open(struct inode *inode, struct file *file)
{
	file->private_data = kxtj2_1009_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

//add by wangyufei begin 20140630
static int yulong_accel_Calibration(struct i2c_client *client, char *buf, int bufsize,int count,unsigned long cal_arg)
{
    int ret = 0;
    //char databuf[6];
    char i;
    //int data[3];
    //char tmp[1];
    int cali[3];
    char strbuf[KXTJ2_1009_BUFSIZE];
    long xavg = 0, yavg = 0, zavg = 0;
    char tempbuf[NAND_FLASH_WR_RD_SIZE];
    //int ret = 0;
    struct acc_offset acc_cal_data;
    struct acc_offset *acc_cal_ptr = &acc_cal_data;

    struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);

    KXTJ2_1009_power(obj->hw, 1);
    msleep(50);
    if((ret = KXTJ2_1009_SetPowerMode(obj->client, true))>0)
    {
        GSE_ERR("write power control fail wangyufei err is %d!!\n",ret);
        //return;
    }


    for (i = 0; i < count; i++)
    {
    if((ret = KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE))>0);
        GSE_ERR("wangyufei I2C error: ret value=%d\n", ret);

        xavg += acc_data[KXTJ2_1009_AXIS_X];
        yavg += acc_data[KXTJ2_1009_AXIS_Y];
        zavg += acc_data[KXTJ2_1009_AXIS_Z];
    }


    printk("(%d),xavg is %ld,Yavg is %ld,Zavg is %ld\n",__LINE__,xavg,xavg,xavg);  
    cali[KXTJ2_1009_AXIS_X] = xavg/count;
    cali[KXTJ2_1009_AXIS_Y] = yavg/count;
    cali[KXTJ2_1009_AXIS_Z] = zavg/count;
    printk("(%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[KXTJ2_1009_AXIS_X],cali[KXTJ2_1009_AXIS_Y],cali[KXTJ2_1009_AXIS_Z]);
    cali[KXTJ2_1009_AXIS_X] = 0-cali[KXTJ2_1009_AXIS_X];
    cali[KXTJ2_1009_AXIS_Y] = 0-cali[KXTJ2_1009_AXIS_Y];
    //modify to pass calibration when mainboard is put the opposite.
   if(-9000 >= cali[KXTJ2_1009_AXIS_Z])                    //wangyufei@yulong.com add for opposite gsensor calilbration begin at 20150601
        cali[KXTJ2_1009_AXIS_Z] = 9807+cali[KXTJ2_1009_AXIS_Z];
    else
         cali[KXTJ2_1009_AXIS_Z] = 9807-cali[KXTJ2_1009_AXIS_Z]; //wangyufei@yulong.com add for opposite gsensor calilbration begin at 20150601
   

    cali[KXTJ2_1009_AXIS_X] = cali[KXTJ2_1009_AXIS_X] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[KXTJ2_1009_AXIS_Y] = cali[KXTJ2_1009_AXIS_Y] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[KXTJ2_1009_AXIS_Z] = cali[KXTJ2_1009_AXIS_Z] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    printk("(%d),calix is %d,caliy is %d,caliz is %d\n",__LINE__,cali[KXTJ2_1009_AXIS_X],cali[KXTJ2_1009_AXIS_Y],cali[KXTJ2_1009_AXIS_Z]);

    //wangyufei@yulong.com modify for acc calibration begin at 20141205
    if(((CAL_FUZZ > cali[KXTJ2_1009_AXIS_X])&&(-CAL_FUZZ < cali[KXTJ2_1009_AXIS_X]))&&((CAL_FUZZ > cali[KXTJ2_1009_AXIS_Y])&&(-CAL_FUZZ < cali[KXTJ2_1009_AXIS_Y]))
        &&((CAL_FUZZ > cali[KXTJ2_1009_AXIS_Z])&&(-CAL_FUZZ < cali[KXTJ2_1009_AXIS_Z])))
    {
        ret = KXTJ2_1009_WriteCalibration(client, cali);
        //GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!xavg =%d,yavg =%d,zavg =%d \n",xavg,yavg,zavg);
        GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!obj->reso->sensitivity = %d,cali[KXTJ2_1009_AXIS_X] =%d,scali[KXTJ2_1009_AXIS_Y] =%d,cali[KXTJ2_1009_AXIS_Z] =%d \n",obj->reso->sensitivity,cali[KXTJ2_1009_AXIS_X],cali[KXTJ2_1009_AXIS_Y],cali[KXTJ2_1009_AXIS_Z]);


        ((struct acc_offset *)acc_cal_ptr)->x = obj->cali_sw[KXTJ2_1009_AXIS_X];
        ((struct acc_offset *)acc_cal_ptr)->y = obj->cali_sw[KXTJ2_1009_AXIS_Y];
        ((struct acc_offset *)acc_cal_ptr)->z = obj->cali_sw[KXTJ2_1009_AXIS_Z];
        ((struct acc_offset *)acc_cal_ptr)->key = ret ? 2 : 1;

        printk("(%d),obj->acc_cali_sw[KXTJ2_1009_AXIS_X] is %d,obj->acc_cali_sw[KXTJ2_1009_AXIS_Y] is %d,obj->acc_cali_sw[KXTJ2_1009_AXIS_Z] is %d\n",
            __LINE__,obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);      

        printk(KERN_INFO "%s write: x = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
        printk(KERN_INFO "%s write: y = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
        printk(KERN_INFO "%s write: z = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->z);
        printk(KERN_INFO "%s write: key = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->key);
        memset(tempbuf, 0, sizeof(tempbuf));

        tempbuf[0]=0x01;
        tempbuf[1] =  (obj->cali_sw[KXTJ2_1009_AXIS_X] & 0xff00) >> 8;
        tempbuf[2] = obj->cali_sw[KXTJ2_1009_AXIS_X] & 0x00ff;
        tempbuf[3] = (obj->cali_sw[KXTJ2_1009_AXIS_Y] & 0xff00) >> 8;
        tempbuf[4] = obj->cali_sw[KXTJ2_1009_AXIS_Y] & 0x00ff;
        tempbuf[5] = (obj->cali_sw[KXTJ2_1009_AXIS_Z] & 0xff00) >> 8;
        tempbuf[6] = obj->cali_sw[KXTJ2_1009_AXIS_Z] & 0x00ff;

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

//add by wangyufei end 20140630

/*----------------------------------------------------------------------------*/
//static int kxtj2_1009_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long kxtj2_1009_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[KXTJ2_1009_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	printk("ACC IOCTL IN WANGYUFEI\n");
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
		//add by wangyufei begin 20140630
		case ACC_CALIBRATE:
			KXTJ2_1009_power(obj->hw, 1);
			msleep(50);
			if((err = KXTJ2_1009_SetPowerMode(obj->client, true))>0)
			{
				GSE_ERR("write power control fail wangyufei err is %ld!!\n",err);
		
			}
			GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
			err = KXTJ2_1009_ResetCalibration(client);	//clear cali
			
			printk("wangyufei acc_calibrate in %s,%d\n",__func__,__LINE__);
            //wangyufei@yulong.com modify for acc calibration begin at 20141205
            err = yulong_accel_Calibration(client, strbuf, KXTJ2_1009_BUFSIZE,20,arg);
            if(0 > err)
            {
                GSE_ERR("accel calibration wangyufei err is %ld!!\n",err);
                err = -EINVAL;
            }
            //wangyufei@yulong.com modify for acc calibration end at 20141205
		  break;

			
		//add by wangyufei end 20140630		
		case GSENSOR_IOCTL_INIT:
			kxtj2_1009_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			KXTJ2_1009_ReadChipInfo(client, strbuf, KXTJ2_1009_BUFSIZE);
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
			KXTJ2_1009_SetPowerMode(obj->client, true);
			KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
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
			KXTJ2_1009_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			GSE_LOG("wangyufei fwq GSENSOR_IOCTL_SET_CALI!!\n");
			yulong_acc_ReadCalibration(client);//add by wangyufei

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
				cali[KXTJ2_1009_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXTJ2_1009_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXTJ2_1009_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = KXTJ2_1009_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = KXTJ2_1009_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = KXTJ2_1009_ReadCalibration(client, cali))>0)
			{
				break;
			}
			
			sensor_data.x = cali[KXTJ2_1009_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[KXTJ2_1009_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[KXTJ2_1009_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
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
    GSE_ERR("accel calibration wangyufei1 err is %ld!!\n",err);    //wangyufei@yulong.com modify for acc calibration  at 20141205
	return err;
}
 

/*----------------------------------------------------------------------------*/
static struct file_operations kxtj2_1009_fops = {
	.owner = THIS_MODULE,
	.open = kxtj2_1009_open,
	.release = kxtj2_1009_release,
	.unlocked_ioctl = kxtj2_1009_unlocked_ioctl,
	//.ioctl = kxtj2_1009_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice kxtj2_1009_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &kxtj2_1009_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
#if 1
static int kxtj2_1009_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);    
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
		if((err = KXTJ2_1009_SetPowerMode(obj->client, false))>0)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}

		//sensor_power = false;      
		KXTJ2_1009_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_resume(struct i2c_client *client)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	KXTJ2_1009_power(obj->hw, 1);
	if((err = kxtj2_1009_init_client(client, 0))>0)
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
static void kxtj2_1009_early_suspend(struct early_suspend *h) 
{
	struct kxtj2_1009_i2c_data *obj = container_of(h, struct kxtj2_1009_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	mutex_lock(&kxtj2_1009_mutex);
	atomic_set(&obj->suspend, 1); 
	if(err = KXTJ2_1009_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}
	mutex_unlock(&kxtj2_1009_mutex);

	//sensor_power = false;
	
	KXTJ2_1009_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void kxtj2_1009_late_resume(struct early_suspend *h)
{
	struct kxtj2_1009_i2c_data *obj = container_of(h, struct kxtj2_1009_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	KXTJ2_1009_power(obj->hw, 1);
	msleep(50);
	if(err = KXTJ2_1009_SetPowerMode(obj->client, TRUE))
	{
		GSE_ERR("write power control fail wangyufei err is %d!!\n",err);
		//return;
	}	
	GSE_ERR("late_resume_ wangyufei err is %d!!\n",err);

/*	mutex_lock(&kxtj2_1009_mutex);
	if(err = kxtj2_1009_init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}

	mutex_unlock(&kxtj2_1009_mutex);
*/	
	atomic_set(&obj->suspend, 0); 
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
/*
static int kxtj2_1009_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, KXTJ2_1009_DEV_NAME);
	return 0;
}
*/

extern int get_device_info(char* buf); //add by wangyufei@yulong.com for device list , 2014.06.26
/*----------------------------------------------------------------------------*/
/*
  * huyue added on 2016/05/25
  */
static int kxtj2_1009_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true

    return 0;
}

/*
 * huyue added on 2016/05/25
 */
static int kxtj2_1009_enable_nodata(int en)
{
    int value = en;
    int err = 0;
    static uint8_t enable_flag = 0;  //shihaobin add read calibration data 20150330
    struct kxtj2_1009_i2c_data *priv = obj_i2c_data;

    if(priv == NULL)
    {
        GSE_ERR("obj_i2c_data is NULL!\n");
        return -1;
    }

    //shihaobin add for read calibration data when first enable sensor begin 20150330
    if (0 == enable_flag)
    {
        enable_flag = 1;
        yulong_acc_ReadCalibration(kxtj2_1009_i2c_client);
    }
    //shihaobin add for read calibration data when first enable sensor end 20150330


    GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);

	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		err = KXTJ2_1009_SetPowerMode( priv->client, !sensor_power);
	}

    GSE_LOG("%s OK!\n",__FUNCTION__);
    return err;
}

/*
 * huyue added on 2016/02/22
 */
static int kxtj2_1009_set_delay(u64 ns)
{
    int value =0;
    int err = 0;
    int sample_delay;
    struct kxtj2_1009_i2c_data *priv = obj_i2c_data;

    value = (int)ns/1000/1000;

    if(priv == NULL)
    {
        GSE_ERR("obj_i2c_data is NULL!\n");
        return -1;
    }

	if(value <= 5)
	{
		sample_delay = KXTJ2_1009_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = KXTJ2_1009_BW_100HZ;
	}
	else
	{
		sample_delay = KXTJ2_1009_BW_50HZ;
	}

	err = KXTJ2_1009_SetBWRate(priv->client, sample_delay);
	if(err != KXTJ2_1009_SUCCESS )
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{
	#if defined(CONFIG_KXTJ2_1009_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[KXTJ2_1009_AXIS_X] = 0;
		priv->fir.sum[KXTJ2_1009_AXIS_Y] = 0;
		priv->fir.sum[KXTJ2_1009_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	#endif
	}

    GSE_LOG("%s (%d), chip only use 1024HZ \n",__FUNCTION__, value);

    return 0;
}


/*
 * huyue added on 2016/05/25
 */
static int kxtj2_1009_get_data(int *x , int *y, int *z, int *status)
{
	char buff[KXTJ2_1009_BUFSIZE];
	int ret;

	KXTJ2_1009_ReadSensorData(obj_i2c_data->client, buff, KXTJ2_1009_BUFSIZE);
	ret = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}


static int kxtj2_1009_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct kxtj2_1009_i2c_data *obj;
	struct hwmsen_object sobj;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	int retry = 0;
	int err = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct kxtj2_1009_i2c_data));

	obj->hw = hw;   //wangyufei@yulong.com modify for gsensor compat at 20141031

	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))>0)
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
	
#ifdef CONFIG_KXTJ2_1009_LOWPASS
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

	kxtj2_1009_i2c_client = new_client;	

	for (retry = 0; retry < 2; retry++)
    {
        err = kxtj2_1009_init_client(new_client, 1);
        if (0 != err)
        {
            GSE_ERR("kxtj2_1009_device init cilent fail time: %d\n", retry);
            continue;
        }
    }
    if (0 != err)
        goto exit_init_failed;

	if((err = misc_register(&kxtj2_1009_device))>0)
	{
		GSE_ERR("kxtj2_1009_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/* modify by mayulong 20130925 begin */
        //#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
        #if 1
	if((err = kxtj2_1009_create_attr(&(kxtj2_1009_init_info.platform_diver_addr->driver))))
        #else
        if(err = kxtj2_1009_create_attr(&kxtj2_1009_gsensor_driver.driver))
        #endif
/* modify by mayulong 20130925 end */
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = kxtj2_1009_open_report_data;
	ctl.enable_nodata = kxtj2_1009_enable_nodata;
	ctl.set_delay  = kxtj2_1009_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;
	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}
	data.get_data = kxtj2_1009_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err= %d\n", err);
		goto exit_kfree;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = kxtj2_1009_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj))>0)
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = kxtj2_1009_early_suspend,
	obj->early_drv.resume   = kxtj2_1009_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

   //begin baibo@yulong.com 20150908 for compatible
   //#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
   kxtj2_1009_init_flag = 0;
   //#endif
  //end baibo@yulong.com 20150908 for compatible

  //begin by baibo@yulong.com 20150915 for device list
  get_device_info("ACC: kxtj2_1009 ROHM \n");
  //end by baibo@yulong.com 20150915 for device list
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&kxtj2_1009_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
   //begin baibo@yulong.com 20150908 for compatible
    //#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
    kxtj2_1009_init_flag = -1;
    //#endif
  //end baibo@yulong.com 20150908 for compatible
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
        /* modify by mayulong 20130925 begin */	
        //#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
        #if 1
	if((err = kxtj2_1009_delete_attr(&(kxtj2_1009_init_info.platform_diver_addr->driver))))
        #else
        if(err = kxtj2_1009_delete_attr(&kxtj2_1009_gsensor_driver.driver))
        #endif       
        /* modify by mayulong 20130925 end */	
	{
		GSE_ERR("kxtj2_1009_delete_attr fail: %d\n", err);
	}
	
	if((err = misc_deregister(&kxtj2_1009_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	//if(err = hwmsen_detach(ID_ACCELEROMETER))
	    

	kxtj2_1009_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
/* modify by mayulong 20130925 begin */ 
//#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
#if 1
static int  kxtj2_1009_remove(void)
{
    //struct acc_hw *hw = kxtj2_get_cust_acc_hw();   //wangyufei@yulong.com modify for gsensor compat at 20141031

    GSE_FUN();    
    KXTJ2_1009_power(hw, 0);    
    i2c_del_driver(&kxtj2_1009_i2c_driver);
    return 0;
}

static int  kxtj2_1009_local_init(void)
{
   //struct acc_hw *hw = kxtj2_get_cust_acc_hw();    //wangyufei@yulong.com modify for gsensor compat at 20141031
	GSE_FUN();
	printk("kxtj2_1009_local_init() \n"); 
	KXTJ2_1009_power(hw, 1);
	if(i2c_add_driver(&kxtj2_1009_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == kxtj2_1009_init_flag)
	{
	   return -1;
	}
	
	return 0;
}
#else
static int kxtj2_1009_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = kxtj2_get_cust_acc_hw();   //wangyufei@yulong.com modify for gsensor compat at 20141031
	GSE_FUN();

	KXTJ2_1009_power(hw, 1);
	//kxtj2_1009_force[0] = hw->i2c_num;
	if(i2c_add_driver(&kxtj2_1009_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = kxtj2_get_cust_acc_hw();   //wangyufei@yulong.com modify for gsensor compat at 20141031

    GSE_FUN();    
    KXTJ2_1009_power(hw, 0);    
    i2c_del_driver(&kxtj2_1009_i2c_driver);
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

static struct platform_driver kxtj2_1009_gsensor_driver = {
	.probe      = kxtj2_1009_probe,
	.remove     = kxtj2_1009_remove,    
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

static struct platform_driver kxtj2_1009_gsensor_driver = {
	.probe      = kxtj2_1009_probe,
	.remove     = kxtj2_1009_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};
#endif
#endif
/* modify by mayulong 20130925 end */ 
/*----------------------------------------------------------------------------*/
static int __init kxtj2_1009_init(void)
{
	const char *name = "mediatek,kxtj2_1009";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&kxtj2_1009_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit kxtj2_1009_exit(void)
{
	GSE_FUN();
#if 0	
/* modify by mayulong 20130925 begin */
        #if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
        //do nothing
        #else
	platform_driver_unregister(&kxtj2_1009_gsensor_driver);
        #endif
/* modify by mayulong 20130925 end */
#endif
}
/*----------------------------------------------------------------------------*/
//module_init(kxtj2_1009_init);
//module_exit(kxtj2_1009_exit);
fs_initcall(kxtj2_1009_init);    //wangyufei@yulong.com modify for gsensor compat at 20141031
module_exit(kxtj2_1009_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KXTJ2_1009 I2C driver");
MODULE_AUTHOR("Dexiang.Liu@mediatek.com");
