/*////////////////////////////////////////////////////////////////////////////////////////////////
//File name		:focal fingerprint sensor driver version 1.0
//Author		:support@focaltech-electronics.com
//Date			:
//Version      	:v1.0
//Description  	:
//ModifyRecord :
*//////////////////////////////////////////////////////////////////////////////////////////////////

#include <linux/io.h>
#include <linux/types.h>
//#include <uapi/linux/spi/spidev.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/spi/spi.h>
//#include <linux/spi/spi_dev.h>
//#include <uapi/linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <asm/io.h>
#include <mt-plat/upmu_common.h>
#include <linux/rtpm_prio.h>   
//#include <asm/arch/mt_gpio.h>

#include <mt-plat/mt_gpio.h>
#include <linux/irqchip/mt-eic.h>
//#include <mach/mt_spi.h>
#include <mt_spi.h>

//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
//#include <cust_eint.h>
//#include <mach/eint.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>



#include "focal_fp_sensor.h"

#if defined(CONFIG_OF)
static struct platform_device *fingerprint_device = NULL;
#endif

/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1

//static struct wake_lock wakeup_wakelock;
#define FOCAL_FP_DEBUG

#ifdef FOCAL_FP_DEBUG
#define FT_DBG(fmt, args...)	printk("focal_fp_sensor" fmt, ##args);
#else
#define FT_DBG(fmt, args...) 	do{}while(0)
#endif

#define MTK_SPI_DMA_MODE
#ifdef MTK_SPI_DMA_MODE
#define MAX_SPI_RW_LEN		1024
#else
#define MAX_SPI_RW_LEN		32
#endif

#define	USE_WAIT_QUEUE	1
#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
//static DECLARE_WAIT_QUEUE_HEAD(waiter);
//static int tpd_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int focal_fp_flag = 0;
struct task_struct *thread_fp = NULL;
#endif
//static u8 g_fp_sensor_rw_buf[MAX_SPI_RW_LEN + 6];

#ifdef FOCAL_FP_SYSFS_DBG
u8 g_captureimage_mode = 0;
#endif

#define FOCAL_ENABLE_FP_SENSOR 		1
#define FOCAL_DISABLE_FP_SENSOR		0
#define FOCAL_FP_FINGER_ON				1		
#define FOCAL_FP_FINGER_OFF			0
#define FOCAL_ENABLE_INTERRUPT_WAKE	1
#define FOCAL_DISABLE_INTERRUPT_WAKE	0

#define TABLENUM	8
#define FOCAL_FP_INTERRUPT          0
#define FOCAL_FP_RESUME                1
#define FOCAL_FP_SUSPEND              2

//#define FOCAL_FP_3V3_EN 		 56//(56 | 0x80000000) //(GPIO56 | 0x80000000)//87
#define FOCAL_SPI_RESET_PIN    	80//(23 | 0x80000000) //(GPIO23 | 0x80000000)//76
#define FOCAL_SPI_INT_PIN    	4

#define CMD_READ_SFR_SINGLE_1 		0x08
#define CMD_READ_SFR_SINGLE_2 		0xf7

//define FOCAL_FP_DETECT_FUNC

static u8 g_enableSensor = FOCAL_ENABLE_FP_SENSOR;
static u8 g_fpfingeron = FOCAL_FP_FINGER_OFF;
static u8 g_fpinterruptwake = FOCAL_ENABLE_INTERRUPT_WAKE;

struct focal_fp_sensor_data {
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device		*spi;
	struct list_head		device_entry;
	struct mutex			buf_lock;
	unsigned				users;
	u8					*buffer;
	unsigned int			standby_irq;
	unsigned int			sleep_irq;
	struct input_dev     	*input;	//report power key event
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend 	focal_early_suspend;
#endif
};

#define FOCAL_FP_DETECT_FUNC


unsigned int ft9308_fingerprint_irq = 0;
static struct pinctrl *pinctrl1;
static struct pinctrl_state *pins_default;
static struct pinctrl_state *eint_as_int, *eint_pulldown, *eint_pulldisable, *rst_output0, *rst_output1;



static struct focal_fp_sensor_data *g_fp_spi_sensor;
static struct focal_fp_rw_operate g_fp_rw_stru;
static u8 *g_rw_buf;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 1024; 

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");							
/*-------------------------------------------------------------------------*/
static int focal_fp_enable_sensor(struct focal_fp_sensor_data *spi_sensor, u8 enable);
#if 1
static void focal_fp_sensor_complete(void *arg)
{
	complete(arg);
}
#endif

#ifdef CONFIG_OF
static struct of_device_id ft9308_of_match[] = {
	{ .compatible = "mediatek,focal_fp_sensor", },
	{},
};
static int fingerprint_dts_probe(struct platform_device *dev)
{
	fingerprint_device = dev;
	printk("nasri...%s %d\n",__func__,__LINE__);

	return 0;
}
static const struct dev_pm_ops fingerprint_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};
static struct platform_driver fingerprint_dts_driver = {
	.probe = fingerprint_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
			.name = "focal_fp_sensor",
			.pm = &fingerprint_pm_ops,
			.owner = THIS_MODULE,
			//.bus	= &spi_bus_type,
			.of_match_table = ft9308_of_match,
			},
};

//MODULE_DEVICE_TABLE(of, ft9308_of_match);
#endif 

static ssize_t
focal_fp_sensor_sync(struct focal_fp_sensor_data *spi_sensor, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
#if 1
	message->complete = focal_fp_sensor_complete;
	message->context = &done;

	spin_lock_irq(&spi_sensor->spi_lock);
	if (spi_sensor->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spi_sensor->spi, message);
	spin_unlock_irq(&spi_sensor->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
#else
	status = spi_sync(spi_sensor->spi, message);
#endif	
	return status;
}

static inline ssize_t
focal_fp_sensor_sync_write(struct focal_fp_sensor_data *spi_sensor, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spi_sensor->buffer,
			.len		= len,
		};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	return focal_fp_sensor_sync(spi_sensor, &m);
}

static inline ssize_t
focal_fp_sensor_sync_read(struct focal_fp_sensor_data *spi_sensor, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spi_sensor->buffer,
			.rx_buf		= spi_sensor->buffer,
			.len		= len,
		};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	return focal_fp_sensor_sync(spi_sensor, &m);
}

static inline ssize_t
focal_fp_sensor_rw(struct focal_fp_sensor_data *spi_sensor, u8 *buf,
u16 buflen)
{
	int err = 0;	
	struct spi_message m;	
	struct spi_transfer t = {			
		.tx_buf	= buf,
		.rx_buf	= buf,
		.len		= buflen,		
	};	
	spi_message_init(&m);	
	spi_message_add_tail(&t, &m);	
	err = focal_fp_sensor_sync(spi_sensor, &m);	
	if( err < 0) {		
		FT_DBG("%s:write error----------\n", __func__);
	}		

	return err;
}
/*
static void focal_fp_sensor_trans_setup(struct spi_message *msg,
			struct spi_transfer *xfers, u8 *tx, u8 *rx, int len,int cs,
			struct focal_fp_sensor_data *spi_sensor)
{

	spi_message_init(msg);
	xfers->cs_change = cs;
	xfers->bits_per_word = spi_sensor->spi->bits_per_word;
	xfers->delay_usecs = 0;
	xfers->speed_hz = spi_sensor->spi->max_speed_hz;
	xfers->len = len;
	xfers->rx_buf = rx;
	xfers->tx_buf = tx;
	spi_message_add_tail(xfers, msg);
}
*/
/* Read-only message with current device setup */
static ssize_t
focal_fp_sensor_client_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct focal_fp_sensor_data	*spi_sensor;
	ssize_t	status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	status = copy_from_user(spi_sensor->buffer, buf, count);
	if (status < 0) {
		return status;
	} else {
	#if 0
		FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, spi_sensor->buffer[0]);
		FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, spi_sensor->buffer[1]);
		FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, spi_sensor->buffer[2]);
		FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, spi_sensor->buffer[3]);
		if (count >=5) {
			FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, spi_sensor->buffer[4]);
			FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, spi_sensor->buffer[5]);
			FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, spi_sensor->buffer[6]);
			FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, spi_sensor->buffer[7]);
		}
	#endif
	}
	status = focal_fp_sensor_sync_read(spi_sensor, count);
	if (status > 0) {
		unsigned long	missing;
		
		missing = copy_to_user(buf, spi_sensor->buffer, status);
		if (missing == status)
		{
			FT_DBG("func:%s line:%d copy_to_user fail \n", __func__, __LINE__);
			status = -EFAULT;
		}
		else
			status = status - missing;
		#if 0
			FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, spi_sensor->buffer[0]);
			FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, spi_sensor->buffer[1]);
			FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, spi_sensor->buffer[2]);
			FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, spi_sensor->buffer[3]);
			if (count >=5) {
			FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, spi_sensor->buffer[4]);
			FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, spi_sensor->buffer[5]);
			FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, spi_sensor->buffer[6]);
			FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, spi_sensor->buffer[7]);

		}
		#endif
	}

	mutex_unlock(&spi_sensor->buf_lock);
	
	if(status > 0)
		status = 0;

	return status;
}

/* Write-only message with current device setup */
static ssize_t
focal_fp_sensor_client_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	unsigned long	missing;
	struct focal_fp_sensor_data	*spi_sensor;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	missing = copy_from_user(spi_sensor->buffer, buf, count);
	if (missing == 0) {
		#if 0
			FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, buf[0]);
			FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, buf[1]);
			FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, buf[2]);
			FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, buf[3]);
			if (count >= 5) {
			FT_DBG("func:%s line:%d buf[0] = %x\n", __func__, __LINE__, buf[4]);
			FT_DBG("func:%s line:%d buf[1] = %x\n", __func__, __LINE__, buf[5]);
			FT_DBG("func:%s line:%d buf[2] = %x\n", __func__, __LINE__, buf[6]);
			FT_DBG("func:%s line:%d buf[3] = %x\n", __func__, __LINE__, buf[7]);
		}
		#endif
		status = focal_fp_sensor_sync_write(spi_sensor, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spi_sensor->buf_lock);

	return status;
}

static inline ssize_t
focal_fp_sensor_read_sfr_register(
	struct focal_fp_sensor_data *spi_sensor, u8 address, u8 *value)
{
	int err = 0;
	u8 status[4]={0x00};
	status[0] = CMD_READ_SFR_SINGLE_1;
	status[1] = CMD_READ_SFR_SINGLE_2;
	status[2] = address;

	struct spi_message m;
	struct spi_transfer t = {
			.tx_buf		= status,
			.rx_buf		= status,
			.len		= 4,
		};
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	//FT_DBG("func:%s line:%d status[0] = %x\n", __func__, __LINE__, status[0]);
	//FT_DBG("func:%s line:%d status[1] = %x\n", __func__, __LINE__, status[1]);
	//FT_DBG("func:%s line:%d status[2] = %x\n", __func__, __LINE__, status[2]);
	//FT_DBG("func:%s line:%d status[3] = %x\n", __func__, __LINE__, status[3]);

	err = focal_fp_sensor_sync(spi_sensor, &m);
	if( err < 0) {
		pr_info("%s:read register ----------\n", __func__);
	}
	*value = status[3];

	//FT_DBG("1 func:%s line:%d status[0] = %x\n", __func__, __LINE__, status[0]);
	//FT_DBG("1 func:%s line:%d status[1] = %x\n", __func__, __LINE__, status[1]);
	//FT_DBG("1 func:%s line:%d status[2] = %x\n", __func__, __LINE__, status[2]);
	//FT_DBG("1 func:%s line:%d status[3] = %x\n", __func__, __LINE__, status[3]);
	
	//pr_info("func=%s[register:0x%02x].[value:0x%02x]\n", __func__, address, status[3]);
	return err;
}

static int focal_fp_sensor_message(struct focal_fp_sensor_data *spi_sensor,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;
	//FT_DBG("%s %d \n",__func__,__LINE__);
	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spi_sensor->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spi_sensor->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}
	status = focal_fp_sensor_sync(spi_sensor, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spi_sensor->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;

				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;
done:
	kfree(k_xfers);
	return status;
}

static void focal_fp_sensor_reset(struct focal_fp_sensor_data *spi_sensor)
{
	#if 0
		//mt_set_gpio_dir(FOCAL_SPI_RESET_PIN, GPIO_DIR_OUT);
		//mt_set_gpio_out(FOCAL_SPI_RESET_PIN, 0);
		gpio_direction_output(FOCAL_SPI_RESET_PIN, 1);
		gpio_set_value(FOCAL_SPI_RESET_PIN, 0);
		mdelay(1);
		//mt_set_gpio_out(FOCAL_SPI_RESET_PIN, 1);
		gpio_set_value(FOCAL_SPI_RESET_PIN, 1);
		mdelay(5);
	#endif

	pinctrl_select_state(pinctrl1, rst_output0);
	msleep(10);
	pinctrl_select_state(pinctrl1, rst_output1);
}

static int focal_fp_lightscreen(void)
{
	input_report_key(g_fp_spi_sensor->input, KEY_POWER, 1);
      	input_sync(g_fp_spi_sensor->input);
	msleep(10);
      	input_report_key(g_fp_spi_sensor->input, KEY_POWER, 0);
      	input_sync(g_fp_spi_sensor->input);
	return 0;
}

static int focal_fp_wakeupSys(void)
{
	input_report_key(g_fp_spi_sensor->input, KEY_WAKEUP, 1);
      	input_sync(g_fp_spi_sensor->input);
	msleep(10);
      	input_report_key(g_fp_spi_sensor->input, KEY_WAKEUP, 0);
      	input_sync(g_fp_spi_sensor->input);
	return 0;
}

static long
focal_fp_sensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//int i = 0; 
	int	err = 0;
	int	retval = 0;
	//short buf_len = 0;
	u32	tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer	*ioc;
	struct focal_fp_sensor_data	*spi_sensor;
	struct spi_device	*spi;

	if (_IOC_TYPE(cmd) != FOCAL_FP_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spi_sensor = filp->private_data;
	spin_lock_irq(&spi_sensor->spi_lock);
	spi = spi_dev_get(spi_sensor->spi);
	spin_unlock_irq(&spi_sensor->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for double duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 */
	mutex_lock(&spi_sensor->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;
	case FOCAL_FP_IOC_GET_SENSOR_STATUS:
		retval = __put_user(g_enableSensor, (__u8 __user *)arg);
		break;
	case FOCAL_FP_IOC_IS_FINGER_ON:
		retval = __put_user(g_fpfingeron, (__u8 __user *)arg);
		break;
	case FOCAL_FP_WRITE_READ_DATA:
		FT_DBG("FOCAL_FP_READ_DATA_______\n");
		if (__copy_from_user(&g_fp_rw_stru, (void __user *)arg, 
			sizeof(struct focal_fp_rw_operate))) {
			retval = -EFAULT;
		}
		if (__copy_from_user(g_rw_buf, (void __user *)g_fp_rw_stru.buf, 
			 g_fp_rw_stru.len)) {
			retval = -EFAULT;
		}
		FT_DBG("____len = %x\n", g_fp_rw_stru.len);
		FT_DBG("____flag = %x\n", g_fp_rw_stru.flag);
		FT_DBG("____buf[0] = %x\n", g_rw_buf[0]);
		FT_DBG("____buf[1] = %x\n", g_rw_buf[1]);
		FT_DBG("____buf[2] = %x\n", g_rw_buf[2]);

		if (g_fp_rw_stru.len > MAX_SPI_RW_LEN) {
			retval = -EFAULT;
			break;
		}
			
		retval = focal_fp_sensor_rw(spi_sensor, g_rw_buf, g_fp_rw_stru.len);
		if(retval < 0) {
			retval = -EFAULT;
			break;
		}
		FT_DBG("_________buf[3] = %x\n", g_rw_buf[3]);
		retval = __copy_to_user((__u8 __user *)g_fp_rw_stru.buf, g_rw_buf, 
			g_fp_rw_stru.len);
		if(retval < 0) {
			FT_DBG("FOCAL_FP_READ_DATA: error copying to user\n");
		}
		break;
	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;
	case FOCAL_FP_IOC_RST_SENSOR:
		FT_DBG("_____FOCAL_FP_IOC_RST_SENSOR____\n");
		focal_fp_sensor_reset(spi_sensor);
		break;
	case FOCAL_FP_IOC_SET_SENSOR_STATUS:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval) {
			retval = focal_fp_enable_sensor(spi_sensor, tmp);
		}
		break;
	case FOCAL_FP_SET_INTERRUPT_WAKE_STATUS:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval) {
			g_fpinterruptwake = tmp;
		}
		break;
	case FOCAL_FP_LIGHTSCREEN:
		focal_fp_lightscreen();
		break;
	case FOCAL_FP_WAKEUP_SYSTEM:
		focal_fp_wakeupSys();
		break;
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;
		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}

		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = focal_fp_sensor_message(spi_sensor, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spi_sensor->buf_lock);
	spi_dev_put(spi);
	
	return retval;
}

#ifdef CONFIG_COMPAT
static long
focal_fp_sensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return focal_fp_sensor_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define focal_fp_sensor_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int focal_fp_sensor_open(struct inode *inode, struct file *filp)
{
	
	int	status = -ENXIO;
	struct focal_fp_sensor_data	*spi_sensor;

	FT_DBG("_______________focal_fp_sensor_open\n");

	mutex_lock(&device_list_lock);
	list_for_each_entry(spi_sensor, &device_list, device_entry) {
		if (spi_sensor->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (spi_sensor->users > 0) {
		mutex_unlock(&device_list_lock);
		FT_DBG(KERN_ERR "Spi sensor: %s: Too many users\n", __func__);
		return -EPERM;
	}
	if (status == 0) {
		if (!spi_sensor->buffer) {
			spi_sensor->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spi_sensor->buffer) {
				dev_dbg(&spi_sensor->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spi_sensor->users++;
			filp->private_data = spi_sensor;
			nonseekable_open(inode, filp);
		}
	} else
		FT_DBG("spi_sensor: nothing for minor %d\n", iminor(inode));
	mutex_unlock(&device_list_lock);
	
	return status;
}


/*
1-enable--leave sleep mode and enter standby mode
0-disable--enter sleep mode for power consumption
*/
static int focal_fp_enable_sensor(struct focal_fp_sensor_data *spi_sensor, u8 enable)
{
	//int err = 0;
	//u8 reg_addr, reg_value = 0x00;
	
	if (FOCAL_DISABLE_FP_SENSOR == enable) {
		
	} else {
		
	}

	g_enableSensor = enable;
	return 0;
}

static void fp_sensor_generate_event(int event)
{
	char *envp[2];

       if (FOCAL_FP_INTERRUPT == event) {
	    envp[0] = "FOCAL=fingeron";
	    envp[1] = NULL;
       } else if (FOCAL_FP_RESUME == event) {
           envp[0] = "FOCAL=resume";
	    envp[1] = NULL;
       } else if (FOCAL_FP_SUSPEND == event) {
           envp[0] = "FOCAL=suspend";
	    envp[1] = NULL;     
       }

	kobject_uevent_env(&g_fp_spi_sensor->spi->dev.kobj, KOBJ_CHANGE, envp);	
}

static irqreturn_t fp_interrupt_handler(int irq, void* handle) {
 //   FT_DBG("__%s %d__\n", __func__, __LINE__);
    focal_fp_flag = 1; 
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

static int focal_fp_event_handler(void *spi_sensor)
{
    //struct focal_fp_sensor_data	*m_spi_sensor = spi_sensor;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

    FT_DBG("focal enter focal_fp_event_handler\n");
    sched_setscheduler(current, SCHED_RR, &param);
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, focal_fp_flag != 0);
        focal_fp_flag = 0;
        set_current_state(TASK_RUNNING);
        FT_DBG("----------------------focal sleep interrupt-----------\n");
        fp_sensor_generate_event(FOCAL_FP_INTERRUPT);
        //mt_eint_unmask(CUST_EINT_FP1_NUM);
    }
    while (!kthread_should_stop());

    return 0;
}

/******************fcoal fingerprint sysfs debug********************/
#ifdef FOCAL_FP_SYSFS_DBG
static struct mutex g_focal_fp_device_mutex;

static ssize_t focalfp_enablesensor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t focalfp_enablesensor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	u8 valbuf[1] = {0};
	u8 enablesensor = 1;
	ssize_t num_read_chars = 0;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct focal_fp_sensor_data *data = spi_get_drvdata(spi);
	
	num_read_chars = count - 1;
	if(num_read_chars > 1) {
		FT_DBG("please input 1 character. 0 or 1----1-enable 0-disable\n");
		return count;
	}
	mutex_lock(&g_focal_fp_device_mutex);
	memcpy(valbuf, buf, num_read_chars);
	//retval = strict_strtoul(valbuf, 10, &enablesensor);
	retval = kstrtoul(valbuf, 10, (long unsigned int *)&enablesensor);

	if (0 != retval) {
		dev_err(&spi->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	} else {
		focal_fp_enable_sensor(data, enablesensor);
	}
error_return:
	mutex_unlock(&g_focal_fp_device_mutex);
	return count;
}

static ssize_t focalfp_captureimage_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//struct spi_device *spi = container_of(dev, struct spi_device, dev);
	//struct focal_fp_sensor_data *data = spi_get_drvdata(spi);
	
	mutex_lock(&g_focal_fp_device_mutex);
	g_captureimage_mode = 1;
	
	g_captureimage_mode = 0;
	mutex_unlock(&g_focal_fp_device_mutex);
	
	return num_read_chars;
}

static ssize_t focalfp_captureimage_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t focalfp_initcfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//struct spi_device *spi = container_of(dev, struct spi_device, dev);
	//struct focal_fp_sensor_data *data = spi_get_drvdata(spi);
	
	mutex_lock(&g_focal_fp_device_mutex);
	mutex_unlock(&g_focal_fp_device_mutex);

	return num_read_chars;
}

static ssize_t focalfp_initcfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static DEVICE_ATTR(focalfpenablesensor, S_IRUGO | S_IWUSR,
			focalfp_enablesensor_show,
			focalfp_enablesensor_store);

static DEVICE_ATTR(focalfpcaptureimage, S_IRUGO | S_IWUSR,
			focalfp_captureimage_show,
			focalfp_captureimage_store);

static DEVICE_ATTR(focalfpinitcfg, S_IRUGO | S_IWUSR,
			focalfp_initcfg_show,
			focalfp_initcfg_store);

/*add your attr in here*/
static struct attribute *focal_fp_attributes[] = {
	&dev_attr_focalfpenablesensor.attr,
	&dev_attr_focalfpcaptureimage.attr,
	&dev_attr_focalfpinitcfg.attr,
	NULL
};

static struct attribute_group focal_fp_attribute_group = {
	.attrs = focal_fp_attributes,
};

/*create sysfs for debug*/
static int focal_fp_create_sysfs(struct spi_device *spi)
{
	int err = 0;
	err = sysfs_create_group(&spi->dev.kobj, &focal_fp_attribute_group);
	if (0 != err) {
		dev_err(&spi->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&spi->dev.kobj, &focal_fp_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_focal_fp_device_mutex);
		FT_DBG("focal_fp:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	
	return err;
}

static void focal_fp_release_sysfs(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &focal_fp_attribute_group);
	mutex_destroy(&g_focal_fp_device_mutex);
}

#endif
/**************************************************************/

static int focal_fp_sensor_release(struct inode *inode, struct file *filp)
{
	int	status = 0;
	struct focal_fp_sensor_data	*spi_sensor;
	
	FT_DBG("________________Spi sensor is closed\n");
	mutex_lock(&device_list_lock);
	spi_sensor = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spi_sensor->users--;
	if (!spi_sensor->users) {
		int	dofree;
		kfree(spi_sensor->buffer);
		spi_sensor->buffer = NULL;
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spi_sensor->spi_lock);
		dofree = (spi_sensor->spi == NULL);
		spin_unlock_irq(&spi_sensor->spi_lock);
		if (dofree)
			kfree(spi_sensor);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations focal_fp_sensor_fops = {
	.owner =	THIS_MODULE,
	.write = focal_fp_sensor_client_write,
	.read = focal_fp_sensor_client_read,
	.unlocked_ioctl = focal_fp_sensor_ioctl,
	.compat_ioctl = focal_fp_sensor_compat_ioctl,
	.open = focal_fp_sensor_open,
	.release = focal_fp_sensor_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

#define FOCAL_FP_DEVICE_NAME "focal_fp"

static struct miscdevice focal_misc_dev =
{
  .minor = MISC_DYNAMIC_MINOR,
  .name = FOCAL_FP_DEVICE_NAME,
  .fops = &focal_fp_sensor_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void focal_fp_suspend(struct early_suspend *h)
{
    fp_sensor_generate_event(FOCAL_FP_SUSPEND);
    msleep(60);
}

static void focal_fp_resume(struct early_suspend *h)
{
    fp_sensor_generate_event(FOCAL_FP_RESUME);
}
#endif

static int fingerprint_get_gpio_info(struct platform_device *pdev)
{
	int ret;
	
	printk("##############[finger pdev->id=%d]+++++++++++++++++\n", pdev->id);
	pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl1!\n");
		return ret;
	}
	pins_default = pinctrl_lookup_state(pinctrl1, "fp_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl default %d!\n", ret);
	}
	
	eint_as_int = pinctrl_lookup_state(pinctrl1, "fp_state_eint_as_int");
	if (IS_ERR(eint_as_int)) {
		ret = PTR_ERR(eint_as_int);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl state_eint_as_int!\n");
		return ret;
	}
	eint_pulldown = pinctrl_lookup_state(pinctrl1, "fp_state_eint_pull_down");
	if (IS_ERR(eint_pulldown)) {
		ret = PTR_ERR(eint_pulldown);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl state_eint_pulldown!\n");
		return ret;
	}
	eint_pulldisable = pinctrl_lookup_state(pinctrl1, "fp_state_eint_pull_disable");
	if (IS_ERR(eint_pulldisable)) {
		ret = PTR_ERR(eint_pulldisable);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl state_eint_pulldisable!\n");
		return ret;
	}
	rst_output0 = pinctrl_lookup_state(pinctrl1, "fp_state_rst_output0");
	if (IS_ERR(rst_output0)) {
		ret = PTR_ERR(rst_output0);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl state_rst_output0!\n");
		return ret;
	}
	rst_output1 = pinctrl_lookup_state(pinctrl1, "fp_state_rst_output1");
	if (IS_ERR(rst_output1)) {
		ret = PTR_ERR(rst_output1);
		dev_err(&pdev->dev, "fwq Cannot find nasri_fingerprint pinctrl state_rst_output1!\n");
		return ret;
	}
	
	pinctrl_select_state(pinctrl1, rst_output1);
	pinctrl_select_state(pinctrl1, eint_as_int);
	
	printk("nasri...%s %d\n",__func__,__LINE__);

	return 0;
}

static int __init focal_fp_sensor_probe(struct spi_device *spi)
{
	int	status=0, ret=0;
	//, err;
	struct focal_fp_sensor_data	*spi_sensor;
	struct input_dev *input_dev = NULL;
	struct device_node *node = NULL;

	//open 3.3V
	//mt_set_gpio_pull_enable(FOCAL_FP_3V3_EN, GPIO_PULL_DISABLE);
    //mt_set_gpio_mode(FOCAL_FP_3V3_EN, GPIO_MODE_GPIO);
    //mt_set_gpio_dir(FOCAL_FP_3V3_EN,GPIO_DIR_OUT);
    //mt_set_gpio_out(FOCAL_FP_3V3_EN,GPIO_OUT_ONE);

#if 0
	if (gpio_request(FOCAL_FP_3V3_EN, "ftsspi_3v3_en") < 0) {
        status = -EBUSY;
        return status;
    }
	gpio_direction_output(FOCAL_FP_3V3_EN, 1);
	gpio_set_value(FOCAL_FP_3V3_EN, 1);

	msleep(50);
#endif

	//reset focal fp sensor
	//mt_set_gpio_dir(FOCAL_SPI_RESET_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(FOCAL_SPI_RESET_PIN, 0);
#if 0
	if (gpio_request(FOCAL_SPI_RESET_PIN, "fts_fp_reset") < 0) {
        status = -EBUSY;
        return status;
    }
	gpio_direction_output(FOCAL_SPI_RESET_PIN, 1);
	gpio_set_value(FOCAL_SPI_RESET_PIN, 0);
	mdelay(1);
	//mt_set_gpio_out(FOCAL_SPI_RESET_PIN, 1);
	gpio_set_value(FOCAL_SPI_RESET_PIN, 1);
	mdelay(5);

	spi_sensor->sleep_irq=gpio_to_irq(FOCAL_SPI_INT_PIN );
#endif
	
	FT_DBG("_________________focal_fp_sensor_probe\n");
	spi_sensor = kzalloc(sizeof(*spi_sensor), GFP_KERNEL);
	if (!spi_sensor)
		return -ENOMEM;

	g_rw_buf = kzalloc(MAX_SPI_RW_LEN, GFP_KERNEL);
	if (!g_rw_buf)
		goto err_free_mem1;
	
	g_fp_spi_sensor = spi_sensor;

	printk("spi_sensor dts ");
	fingerprint_get_gpio_info(fingerprint_device);

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&spi->dev, "Failed to allocate memory\n");
		status = -ENOMEM;
		goto err_free_mem2;
	}

	input_dev->name = "focal_fp_btn";
	spi_sensor->input = input_dev;

	//__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	//__set_bit(BTN_TOUCH, input_dev->keybit);
	//__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);

	status = input_register_device(input_dev);
	if (status) {
		dev_err(&spi->dev,
			"%s: failed to register input device: %s\n",
			__func__, dev_name(&spi->dev));
		goto exit_input_register_device_failed;
	}

	spi->bits_per_word = 8;
	spi_sensor->spi = spi;
	spin_lock_init(&spi_sensor->spi_lock);
	mutex_init(&spi_sensor->buf_lock);

	focal_fp_sensor_reset(spi_sensor);

	mdelay(2000);

#if 1
	printk("====================================\n");
	u8 buf2;
	focal_fp_sensor_read_sfr_register(spi_sensor, 0x10, &buf2);
	printk("===0x10  %d===\n", buf2);
	focal_fp_sensor_read_sfr_register(spi_sensor, 0x14, &buf2);
	printk("===0x14  %d===\n", buf2);
#endif

	thread_fp = kthread_run(focal_fp_event_handler, 0, "focal_fp");

	pinctrl_select_state(pinctrl1, eint_pulldisable);

	node = of_find_matching_node(node, ft9308_of_match);
	if (node) {
		ft9308_fingerprint_irq = irq_of_parse_and_map(node, 0);
		printk("irq_of_parse_and_map---fingerprint_irq=%d\n   ",ft9308_fingerprint_irq);
		
		ret = request_irq(ft9308_fingerprint_irq, (irq_handler_t) fp_interrupt_handler, IRQF_TRIGGER_RISING,"FINGERPRINT-eint", NULL);
		printk("ft9308_fingerprint_irq ret=%d\n   ",ret);
	}

	//disable_irq(ft9308_fingerprint_irq);

	//ret = request_irq(g_fp_spi_sensor->sleep_irq, fp_interrupt_handler, 
        //IRQF_TRIGGER_RISING, "focal_fp", NULL);
   	 //FT_DBG("2___irq = %d__ ret = %d\n", g_fp_spi_sensor->sleep_irq, ret);
	INIT_LIST_HEAD(&spi_sensor->device_entry);
	
	mutex_lock(&device_list_lock);
	
	spi_sensor->devt = MKDEV(MISC_MAJOR, focal_misc_dev.minor);
	list_add(&spi_sensor->device_entry, &device_list);
	
	mutex_unlock(&device_list_lock);
	if (status == 0)
		spi_set_drvdata(spi, spi_sensor);
	else
		kfree(spi_sensor);
	
#ifdef FOCAL_FP_SYSFS_DBG
	focal_fp_create_sysfs(spi);
#endif
	FT_DBG("_________________probe end\n");
	return status;
	
#ifdef FOCAL_FP_DETECT_FUNC
//exit_standby_irq_request_failed:
	//free_irq(spi_sensor->standby_irq, spi_sensor);
#endif
exit_input_register_device_failed:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_mem2:
	kfree(g_rw_buf);
err_free_mem1:
	kfree(spi_sensor);

	return status;
}

//static int __devexit focal_fp_sensor_remove(struct spi_device *spi)
static int focal_fp_sensor_remove(struct spi_device *spi)
{
	struct focal_fp_sensor_data	*spi_sensor = (struct focal_fp_sensor_data *)spi_get_drvdata(spi);

#ifdef FOCAL_FP_DETECT_FUNC
	free_irq(spi_sensor->standby_irq, spi_sensor);
	free_irq(spi_sensor->sleep_irq, spi_sensor);
#endif
#ifdef FOCAL_FP_SYSFS_DBG
	focal_fp_release_sysfs(spi_sensor->spi);
#endif

	input_unregister_device(spi_sensor->input);
	spi_sensor->input = NULL;

	//unregister_early_suspend(&spi_sensor->early_suspend);
	
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spi_sensor->spi_lock);
	spi_sensor->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_sensor->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spi_sensor->device_entry);
	//clear_bit(MINOR(spi_sensor->devt), minors);
	//kfree(g_rw_user_buf);
	kfree(g_rw_buf);
	if (spi_sensor->users == 0)
		kfree(spi_sensor);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver focal_fp_sensor_driver = {
	.driver = {
		.name =		"focal_fp_sensor",
		.owner =	THIS_MODULE,
	},
	.probe =	focal_fp_sensor_probe,
	.remove = focal_fp_sensor_remove,//__devexit_p(focal_fp_sensor_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.
	 */
};

#define SPI_DMA_MODE

static struct mt_chip_conf focal_chip_config = {
	.setuptime = 50,
	.holdtime = 50,
	.high_time = 20,
	.low_time = 20,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = PAUSE_MODE_ENABLE,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
			.modalias="focal_fp_sensor",
			.bus_num = 0,
			.chip_select=0,
			.mode = SPI_MODE_0,
			.controller_data = &focal_chip_config,
	},
};

/*-------------------------------------------------------------------------*/
static int __init focal_fp_sensor_init(void)
{
	int status;
	
	FT_DBG("_________________focal_fp_sensor_init\n");
	status = misc_register(&focal_misc_dev);
	if (status < 0) {
		FT_DBG("focaltech:misc_register error!\n");
		return status;
	}

	status = platform_driver_register(&fingerprint_dts_driver);
	printk("platform_driver_register---status=%d\n",status);

	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));

	status = spi_register_driver(&focal_fp_sensor_driver);
	if (status < 0) {
		FT_DBG("focaltech:spi_register_driver error!\n");
		return status;
	}
		
	return 0;
}
module_init(focal_fp_sensor_init);

static void __exit focal_fp_sensor_exit(void)
{
	spi_unregister_driver(&focal_fp_sensor_driver);
	misc_deregister(&focal_misc_dev);
}
module_exit(focal_fp_sensor_exit);


MODULE_AUTHOR("support@focaltech-electronics.com");
MODULE_DESCRIPTION("User mode fp sensor device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:focal_fp_sensor");
