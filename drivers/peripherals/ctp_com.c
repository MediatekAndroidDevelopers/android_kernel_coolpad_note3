#include <linux/errno.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <mach/peripherals_com.h>



static struct mutex	 ctp_mutex;
static int mutex_mark=-1;

int fm_poweron_status = 0;//FALSE
int tp_is_sleep = 0;
int mxc_ldo_sim2_turnon_times= 0;
int tp_ldo_sim2_turnon_times= 0;
int fm_ldo_sim2_turnon_times = 0;
int tp_is_gt9xx = 0;
EXPORT_SYMBOL_GPL(fm_poweron_status);
EXPORT_SYMBOL_GPL(tp_is_sleep);
EXPORT_SYMBOL_GPL(mxc_ldo_sim2_turnon_times);
EXPORT_SYMBOL_GPL(tp_ldo_sim2_turnon_times);
EXPORT_SYMBOL_GPL(fm_ldo_sim2_turnon_times);
EXPORT_SYMBOL_GPL(tp_is_gt9xx);

void ctp_lock_mutex(void)
{
    if(-1==mutex_mark)
    {
        mutex_mark=1;
        mutex_init(&ctp_mutex);
    }
    mutex_lock(&ctp_mutex);
}

void ctp_unlock_mutex(void)
{
    if(1==mutex_mark)
    {
		mutex_unlock(&ctp_mutex);
    }
}

int tp_device_id(int id)
{
    static int device_ctp_id=0;
    unsigned long flags;
    local_irq_save(flags);
    
    if(id!=0)
    {
        device_ctp_id=id;
    }
    if(id==0xFFFF)
    {
        device_ctp_id=0;
    }
    local_irq_restore(flags);
    //CTP_DEBUG("ctp chip id(0x%x)",device_ctp_id);
    return device_ctp_id;
}

EXPORT_SYMBOL_GPL(ctp_lock_mutex);
EXPORT_SYMBOL_GPL(ctp_unlock_mutex);
EXPORT_SYMBOL_GPL(tp_device_id);
/*
static int fm_id=0;
int fm_device_id(int id)
{
	unsigned long	flags;
	local_irq_save(flags);

	if(id!=0)
	{
		fm_id=id;
	}
	if(id==0xFFFF)
	{
		fm_id=0;
	}
	local_irq_restore(flags);
	printk("FM Chip Id(0x%x)", fm_id);
	return fm_id;
}
EXPORT_SYMBOL_GPL(fm_device_id);
*/