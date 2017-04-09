#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
//#include <cust_gpio_usage.h>
//#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/gpio_const.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

//static int g_bLtVersion=0;
// ***********************MT6580 M  test wsl*************
#if 1//strong flash
#define DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT
#define __FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__
#else //weak flash
#define DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT
#define __FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__
#endif
//************************MT6580 M  test end**************

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);



#if defined(DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

#if defined(__FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)


#define GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN 43

#ifndef GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN
#error Please define GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN
#endif

static inline int drt_weak_current_flashlight_switch_by_gpio(bool switch_on)
{
	//printk("[wsl] drt_weak_current_flashlight_switch_by_gpio start");
	
	if (switch_on)
	{
		//mt_set_gpio_out(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, GPIO_OUT_ONE);
		gpio_set_value(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, 1);
	}
	else
	{
		//mt_set_gpio_out(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, GPIO_OUT_ZERO);
		gpio_set_value(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, 0);
	}
	
	return 0;
}
#endif

#elif defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

#define MODE_FLASHLIGHT    1
#define MODE_TORCH         0
#define CUST_YUV_SENSOR_FLASHLIGHT_DUTY 	24
#define CUST_YUV_SENSOR_TORCH_DUTY 	12

#define CUST_RAW_SENSOR_FLASHLIGHT_DUTY 	1
#define CUST_RAW_SENSOR_TORCH_DUTY 	0

static char flashlight_mode = MODE_TORCH;

#if defined(__FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)

//#define GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN (GPIO7 | 0x80000000)
//#define GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN (GPIO8 | 0x80000000)

#define GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN 43
#define GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN 42

#ifndef GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN
#error Please define GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN
#endif

#ifndef GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN
#error Please define GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN
#endif

static inline int drt_strong_current_flashlight_switch_by_gpio(bool switch_on, char mode)

{
	//printk("[wsl] drt_strong_current_flashlight_switch_by_gpio start");
	// flashlight mode
	switch (mode)
	{
	case MODE_FLASHLIGHT:
		gpio_set_value(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN, 1);
		break;
	default://TORCH MODE
		gpio_set_value(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN, 0);
		break;
	}
	
	// flashlight switch
	if (switch_on)
	{
		gpio_set_value(GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN, 1);
	}
	else
	{
		gpio_set_value(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN, 0);
		gpio_set_value(GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN, 0);
	}
	//printk("[wsl]mode = %d , switch_on = %d \n" ,mode, switch_on);
	return 0;
}
#endif

#else
// Some project don't assemble flashlight, so we comment the following code.
//#error please reconfig flashlight or to edit code.
#endif

int FL_Enable(void)
{
	int ret = 0;
	
#if defined(DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

	#if defined(__FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	//printk("[wsl] FL_ENABLE by __FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__");
	ret |= drt_weak_current_flashlight_switch_by_gpio(true);
	#endif
	

#elif defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)
	#if defined(__FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	//printk("[wsl] FL_ENABLE by __FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__");
	ret = drt_strong_current_flashlight_switch_by_gpio(true, flashlight_mode);
	#endif
#endif

    return ret;
}



int FL_Disable(void)
{
	int ret = 0;
	
#if defined(DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

	#if defined(__FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	//printk("[wsl] FL_DISABLE by __FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__ \n");
	ret |= drt_weak_current_flashlight_switch_by_gpio(false);
	#endif
	
#elif defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)
	#if defined(__FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	//printk("[wsl] FL_DISABLE by __FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__ \n");
	ret = drt_strong_current_flashlight_switch_by_gpio(false, flashlight_mode);
	#endif
#endif

    return ret;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;


#if defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)//wangsl for sc 

	if (CUST_RAW_SENSOR_FLASHLIGHT_DUTY == duty || CUST_YUV_SENSOR_FLASHLIGHT_DUTY == duty)
	{	
		flashlight_mode = MODE_FLASHLIGHT;
	}
	else
	{
		flashlight_mode = MODE_TORCH;
	}

#endif

    return 0;
}




int FL_Init(void)
{
#if defined(DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

	#if defined(__FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)

			gpio_request(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, "flash_weak_en");
			gpio_direction_output(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN, 0);
	#endif
	
#elif defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)

	#if defined(__FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
			printk("[wsl] FL_Init start gpio_request and gpio_direction_output \n");
			gpio_request(GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN, "flash_strong_en");
			gpio_request(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN, "flash_strong_mode");
			gpio_direction_output(GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN, 0);
			gpio_direction_output(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN, 0);
	#endif
#endif

	INIT_WORK(&workTimeOut, work_timeOutFunc);
 
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	
#if defined(DRT_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_SUPPORT)
	#if defined(__FUNC_DRV_WEAK_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	gpio_free(GPIO_WC_MAIN_FLASHLIGHT_SWITCH_PIN);
	#endif
#elif defined(DRT_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_SUPPORT)
	#if defined(__FUNC_DRV_STRONG_CURRENT_MAIN_FLASHLIGHT_GPIO_DRT__)
	gpio_free(GPIO_SC_MAIN_FLASHLIGHT_SWITCH_PIN);
	gpio_free(GPIO_SC_MAIN_FLASHLIGHT_MODE_PIN);
	printk("[wsl] FL_Uninit start gpio_free \n");
	#endif
#endif
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
