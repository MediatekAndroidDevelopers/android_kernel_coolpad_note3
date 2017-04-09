//#include <linux/init.h>
//#include <linux/module.h>
//#include <linux/types.h>
#include <linux/fs.h>
//#include <linux/proc_fs.h>
#include <linux/device.h>
//#include <linux/slab.h>
#include <asm/uaccess.h>

#include <linux/capability.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/socket.h>
#include <linux/in.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/sockios.h>
#include <linux/net.h>
#include <linux/slab.h>
#include <net/ax25.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <asm/uaccess.h>
#include <linux/fcntl.h>
#include <linux/termios.h>	/* For TIOCINQ/OUTQ */
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/netfilter.h>
#include <linux/sysctl.h>
#include <linux/init.h>
#include <linux/spinlock.h>


//----------------------------------
#include "zyt_peripherals.h"

#include <mach/peripherals_com.h>

void Sensor_GetSensor_Index_info(int sensorindex, char * sensor_name_index,int *true_false);
void TP_GetTP_Name_info(char * tp_name);

void modem_get_band_string(void);

/*主设备和从设备号变量*/
static int zyt_peripherals_major = 0;
static int zyt_peripherals_minor = 0;

/*设备类别和设备变量*/
static struct class* zyt_peripherals_class = NULL;
static struct zyt_peripherals_dev* peripherals_dev = NULL;
//static struct proc_dir_entry *entry = NULL;


/*传统的设备文件操作方法*/
static int zyt_peripherals_open(struct inode* inode, struct file* filp);
static int zyt_peripherals_release(struct inode* inode, struct file* filp);
static ssize_t zyt_peripherals_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos);
static ssize_t zyt_peripherals_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos);

/*设备文件操作方法表*/
static struct file_operations zyt_peripherals_fops = {
	.owner = THIS_MODULE,
	.open = zyt_peripherals_open,
	.release = zyt_peripherals_release,
	.read = zyt_peripherals_read,
	.write = zyt_peripherals_write, 
};

/*访问设置属性方法*/
static ssize_t zyt_peripherals_val_show(struct device* dev, struct device_attribute* attr,  char* buf);
static ssize_t zyt_peripherals_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);

/*定义设备属性*/
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, zyt_peripherals_val_show, zyt_peripherals_val_store);
//----------------------------------------------------------------------------------

/*打开设备方法*/
static int zyt_peripherals_open(struct inode* inode, struct file* filp) {
	struct zyt_peripherals_dev* dev;        
	
	/*将自定义设备结构体保存在文件指针的私有数据域中，以便访问设备时拿来用*/
	dev = container_of(inode->i_cdev, struct zyt_peripherals_dev, dev);
	filp->private_data = dev;
	
	return 0;
}

/*设备文件释放时调用，空实现*/
static int zyt_peripherals_release(struct inode* inode, struct file* filp) {
	return 0;
}

/*读取设备的寄存器val的值*/
static ssize_t zyt_peripherals_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos) {
	ssize_t err = 0;
	struct zyt_peripherals_dev* dev = filp->private_data;        

	/*同步访问*/
	if(down_interruptible(&(dev->sem))) {
		return -ERESTARTSYS;
	}

	if(count < sizeof(dev->val)) {
		goto out;
	}        

	/*将寄存器val的值拷贝到用户提供的缓冲区*/
	if(copy_to_user(buf, &(dev->val), sizeof(dev->val))) {
		err = -EFAULT;
		goto out;
	}

	err = sizeof(dev->val);

out:
	up(&(dev->sem));
	return err;
}

/*写设备的寄存器值val*/
static ssize_t zyt_peripherals_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos) {
	struct zyt_peripherals_dev* dev = filp->private_data;
	ssize_t err = 0;        

	/*同步访问*/
	if(down_interruptible(&(dev->sem))) {
		return -ERESTARTSYS;        
	}        

	if(count != sizeof(dev->val)) {
		goto out;        
	}        

	/*将用户提供的缓冲区的值写到设备寄存器去*/
	if(copy_from_user(&(dev->val), buf, count)) {
		err = -EFAULT;
		goto out;
	}

	err = sizeof(dev->val);

out:
	up(&(dev->sem));
	return err;
}
#ifdef USE_ZYT_PERIPHERALS
//----------------------------------------------------------------------------------
static int LCD_main_flag = 0;
char LCD_MAIN[32];
static char CAMERA_MAIN[32]={0};

static int CAMERA_main_flag=0;

static char CAMERA_SUB[32]={0};
static int CAMERA_sub_flag = 0;

static int flash_size_flag = 0;
static char flash_size[32] = {0};

static int platform_flag = 0;
static char platform[32] = {0};

static char TP_NAME[32]={0};

static char old_peripherals_info[512] = {0};
static int b_has_got_peripherals_info = 0;
static int b_has_read_peripherals_info = 0;

//解析cmdline文件信息
static int parse_cmdline_info(void)
{
	char *lcd_info_keywords = "lcm=1-";
	char *flash_info_keywords = "nandflash=nandid(";
	
	char *temp = NULL;
	char *search_flag = NULL;
	
	char buf[32] = {0};
	int i = 0;

	/*
mtk
	console=tty0 console=ttyMT0,921600n1 root=/dev/ram vmalloc=500M slub_max_order=0
 slub_debug=O  lcm=1-r61581b_mcu_6572_hvga fps=6672 pl_t=1581 lk_t=1503 printk.d
isable_uart=0 boot_reason=0
	*/

	//获得LCD 信息
	search_flag = strstr(saved_command_line, lcd_info_keywords);
	//printk(KERN_ALERT"Search lcd_info_keywords return: %lu %lu.\n", search_flag, saved_command_line);

	if(NULL == search_flag)
	{
		lcd_info_keywords = "lcm=0-";
		search_flag = strstr(saved_command_line, lcd_info_keywords);
	}
		
	if(NULL != search_flag)
	{
		temp = search_flag + strlen(lcd_info_keywords);
		//printk(KERN_ALERT"temp: %lu.\n", temp);
		
		while('f' != *temp)
		{
			buf[i++] = *temp;
			temp++;
		}
		//printk(KERN_ALERT"Get LCD_ID str: %s.\n", buf);
		strncpy(LCD_MAIN,buf,strlen(buf));
		
		//printk(KERN_ALERT"LCD_MAIN: 0x%x.\n", LCD_MAIN);
		LCD_main_flag=1;
	}
		
	//Get flash size
	search_flag = strstr(saved_command_line, "flash_size=");
	if (NULL != search_flag)
	{
		temp = search_flag + strlen("flash_size=");
		//printk(KERN_ALERT"zhu temp: %lu.\n", temp);
		i = 0;
		
		do
		{
			buf[i++] = *temp;
		}while (' ' != *temp++);
		
		buf[i] = 0;
		
		//printk(KERN_ALERT"zhu Get mem size str: %s.\n", buf);
		strncpy(flash_size, buf, strlen(buf));
		
		//printk(KERN_ALERT"zhu flash_size: 0x%x.\n", flash_size);
		flash_size_flag = 1;
	}

	//Get platform
	search_flag = strstr(saved_command_line, "platform=");
	if (NULL != search_flag)
	{
		temp = search_flag + strlen("platform=");
		//printk(KERN_ALERT"zhu temp: %lu.\n", temp);
		i = 0;
		
		do
		{
			buf[i++] = *temp;
		}while (' ' != *temp++);

		buf[i] = 0;
		
		//printk(KERN_ALERT"zhu Get platform str: %s.\n", buf);
		strncpy(platform, buf, strlen(buf));
		
		//printk(KERN_ALERT"zhu platform: 0x%x.\n", platform);
		platform_flag = 1;
	}
	
	return 0;
}

/*通过id获取设备名字，内部使用*/
static int __zyt_peripherals_get_name_by_id(void *id, char *name, E_ZYT_PERIPHERALS_TYPE type)
{
	int i = 0;
	
	if(NULL == name)
		return 0;

	switch(type)
	{
		case TYPE_TP:
			{
				int total_currType_num = sizeof(zyt_peripherals_TP_info)/sizeof(PERIPHERALS_ID_2_NAME);

				//  printk("TYPE_TP id======%d\n",(int)id);
				
				for(i = 0; i < total_currType_num; i++)
				{
					if((int)id == zyt_peripherals_TP_info[i].id)
					{
						sprintf(name, "[TP]%s", zyt_peripherals_TP_info[i].name);
						break;
					}
				}

				if(i == total_currType_num)
				{
					sprintf(name, "[TP]%s", zyt_peripherals_TP_info[i-1].name);
				}
				if(strncmp(name,"Unknown",sizeof("Unknown")))
				{
					sprintf(name, "[TP]%s", TP_NAME);
				}
			}
			break;

		case TYPE_LCD:
			{
					if((int)id  ==1)
					{
						sprintf(name, "[LCD]%s", LCD_MAIN);
						break;
					}
					else
						sprintf(name, "[LCD]%s", ZYT_PERIPHERALS_UNKNOWN);
			}
			break;

		case TYPE_CAMERA_MAIN:
			{
				if(0 == (int)id)
				{
					sprintf(name , "[CAMERA_MAIN]%s", ZYT_PERIPHERALS_UNKNOWN);
				}
				else
				{
					sprintf(name , "[CAMERA_MAIN]%s",CAMERA_MAIN );
				}
			}
			break;

		case TYPE_CAMERA_SUB:
			{
				if(0 == (int)id)
				{
					sprintf(name , "[CAMERA_SUB]%s", ZYT_PERIPHERALS_UNKNOWN);
				}
				else
				{
					sprintf(name , "[CAMERA_SUB]%s", CAMERA_SUB);
      
				}
			}
			break;
			
		case TYPE_FALSH:
		{
				if(0 == (int)id)
				{
					sprintf(name , "[FLASH_SIZE]%s", ZYT_PERIPHERALS_UNKNOWN);
				}
				else
				{
					sprintf(name , "[FLASH_SIZE]%s", flash_size);
				}
		}

		break;

		case TYPE_PLATFORM:
		{
				if(0 == (int)id)
				{
					sprintf(name , "[PLATFORM]%s", ZYT_PERIPHERALS_UNKNOWN);
				}
				else
				{
					sprintf(name , "[PLATFORM]%s", platform);
				}
		}

		break;

		default:
			break;
			
	}
	
	//printk(KERN_ALERT"strlen(name): %d.\n", strlen(name));		
	return strlen(name);
}
/*读取寄存器peripherals_info的值到缓冲区buf中，内部使用*/
static ssize_t __zyt_peripherals_get_peripherals_info(struct zyt_peripherals_dev* dev, char* buf) {
	int len = 0;
	int ascending_len = 0;

	/*同步访问*/
	
	//if(down_interruptible(&(dev->sem))) {                
	//	return -ERESTARTSYS;        
	//}    

	//modem_get_band_string();
	
	if(0 == b_has_got_peripherals_info)
	{
		printk(KERN_ALERT"First time to get peripherals info.\n");	
		
		//清空数据
		memset(dev->peripherals_info, 0, sizeof(dev->peripherals_info));
		memset(old_peripherals_info, 0, sizeof(old_peripherals_info));


		//从cmdline获取各个设备信息
		parse_cmdline_info();

		//for TP name
		TP_GetTP_Name_info(TP_NAME);
		len = __zyt_peripherals_get_name_by_id((void*)tp_device_id(0), dev->peripherals_info, TYPE_TP);
		ascending_len += len;
       	dev->peripherals_info[ascending_len++] = ';';

		//for LCD name
		len = __zyt_peripherals_get_name_by_id((void*)LCD_main_flag, dev->peripherals_info+ascending_len, TYPE_LCD);
		ascending_len += len;
		dev->peripherals_info[ascending_len++] = ';';

		//for main CAMERA name
		
		Sensor_GetSensor_Index_info(0,CAMERA_MAIN,&CAMERA_main_flag);
	//	printk("main camera 22222222222=======%d ,name ======%s\n",CAMERA_main_flag,CAMERA_MAIN);
		len = __zyt_peripherals_get_name_by_id((void*)CAMERA_main_flag, dev->peripherals_info+ascending_len, TYPE_CAMERA_MAIN);
		ascending_len += len;
		dev->peripherals_info[ascending_len++] = ';';
		//for sub CAMERA name
		Sensor_GetSensor_Index_info(1,CAMERA_SUB,&CAMERA_sub_flag);
		//printk("main camera 3333333333333333=======%d ,name ======%s\n",CAMERA_sub_flag,CAMERA_SUB);
		len = __zyt_peripherals_get_name_by_id((void*)CAMERA_sub_flag, dev->peripherals_info+ascending_len, TYPE_CAMERA_SUB);
		ascending_len += len;
		dev->peripherals_info[ascending_len++] = ';';

		//for FLASH SIZE
		len = __zyt_peripherals_get_name_by_id((void*)flash_size_flag, dev->peripherals_info+ascending_len, TYPE_FALSH);
		ascending_len += len;
		dev->peripherals_info[ascending_len++] = ';';

		//for PLATFORM
		len = __zyt_peripherals_get_name_by_id((void*)platform_flag, dev->peripherals_info+ascending_len, TYPE_PLATFORM);
		ascending_len += len;
		dev->peripherals_info[ascending_len++] = ';';

		//保存旧数据
		//b_has_got_peripherals_info = 1;
		strncpy(old_peripherals_info, dev->peripherals_info, strlen(dev->peripherals_info));
	}
	else
	{
		printk(KERN_ALERT"Return old peripherals info.\n");	
		strncpy(dev->peripherals_info, old_peripherals_info, strlen(old_peripherals_info));
	}

//	up(&(dev->sem));        

	return snprintf(buf, PAGE_SIZE, "%s\n", dev->peripherals_info);
}
#endif

/*读取寄存器val的值到缓冲区buf中，内部使用*/
static ssize_t __zyt_peripherals_get_val(struct zyt_peripherals_dev* dev, char* buf) {
	int val = 0;    

	/*同步访问*/
	if(down_interruptible(&(dev->sem))) {                
		return -ERESTARTSYS;        
	}        
	
	val = dev->val;   
	up(&(dev->sem));	   

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

/*把缓冲区buf的值写到设备寄存器val中去，内部使用*/
static ssize_t __zyt_peripherals_set_val(struct zyt_peripherals_dev* dev, const char* buf, size_t count) {
	int val = 0;        

	/*将字符串转换成数字*/        
	val = simple_strtol(buf, NULL, 10);        

	/*同步访问*/        
	if(down_interruptible(&(dev->sem))) {                
		return -ERESTARTSYS;        
	}        

	dev->val = val;        
	up(&(dev->sem));

	return count;
}

/*读取设备属性val*/
static ssize_t zyt_peripherals_val_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct zyt_peripherals_dev* hdev = (struct zyt_peripherals_dev*)dev_get_drvdata(dev);        

	#ifdef USE_ZYT_PERIPHERALS
	return __zyt_peripherals_get_peripherals_info(hdev, buf);
	#else
	return __zyt_peripherals_get_val(hdev, buf);
	#endif
}

/*写设备属性val*/
static ssize_t zyt_peripherals_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) { 
	struct zyt_peripherals_dev* hdev = (struct zyt_peripherals_dev*)dev_get_drvdata(dev);  
	
	return __zyt_peripherals_set_val(hdev, buf, count);
}
//----------------------------------------------------------------------------------

#if 0

ssize_t zyt_peripherals_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos){

	char *peripherals_page_buffer[PAGE_SIZE];
	size_t read_size;

	printk(KERN_ALERT"zyt_peripherals_proc_read start");
	#ifdef USE_ZYT_PERIPHERALS
	read_size = __zyt_peripherals_get_peripherals_info(peripherals_dev, peripherals_page_buffer);
	#else
	read_size = __zyt_peripherals_get_val(peripherals_dev, peripherals_page_buffer);
	#endif

	copy_to_user(buf, peripherals_page_buffer, read_size);

	 *ppos = read_size;

	 printk(KERN_ALERT"zyt_peripherals_proc_read end");
	 return read_size;
}

#else
/*读取设备寄存器val的值，保存在page缓冲区中*/
static ssize_t zyt_peripherals_proc_read(struct file* filp, char __user *page, size_t count, loff_t* f_pos) {
	int ret = 0;  
	printk(KERN_ALERT"zyt_peripherals_proc_read b_has_read_peripherals_info = %d.\n",b_has_read_peripherals_info);  
	if(b_has_read_peripherals_info > 0) {
		b_has_read_peripherals_info = 0;
		return 0;
	}
	
	#ifdef USE_ZYT_PERIPHERALS
	ret = __zyt_peripherals_get_peripherals_info(peripherals_dev, page);
	#else
	ret = __zyt_peripherals_get_val(peripherals_dev, page);
	#endif
	//b_has_read_peripherals_info = 1;
	return ret;
}

#endif

/*把缓冲区的值buff保存到设备寄存器val中去*/
static ssize_t zyt_peripherals_proc_write(struct file* filp, const char __user *buff, unsigned long len, void* data) {
	int err = 0;
	char* page = NULL;

	if(len > PAGE_SIZE) {
		printk(KERN_ALERT"The buff is too large: %lu.\n", len);
		return -EFAULT;
	}

	page = (char*)__get_free_page(GFP_KERNEL);
	if(!page) {                
		printk(KERN_ALERT"Failed to alloc page.\n");
		return -ENOMEM;
	}        

	/*先把用户提供的缓冲区值拷贝到内核缓冲区中去*/
	if(copy_from_user(page, buff, len)) {
		printk(KERN_ALERT"Failed to copy buff from user.\n");                
		err = -EFAULT;
		goto out;
	}

	err = __zyt_peripherals_set_val(peripherals_dev, page, len);

out:
	free_page((unsigned long)page);
	return err;
}

static const struct file_operations zyt_peripherals_proc_fops = { 
    .read = zyt_peripherals_proc_read,
    .write =zyt_peripherals_proc_write
};

/*创建/proc/zyt_peripherals文件*/
static struct proc_dir_entry *entry = NULL;
static void zyt_peripherals_create_proc(void) {
	//struct proc_dir_entry* entry;
	
	//entry = create_proc_entry(ZYT_PERIPHERALS_DEVICE_PROC_NAME, 0, NULL);
	entry = proc_create(ZYT_PERIPHERALS_DEVICE_PROC_NAME, 0, NULL, &zyt_peripherals_proc_fops);
	//if(entry) {
	//	entry->owner = THIS_MODULE;
	//	entry->read_proc = zyt_peripherals_proc_read;
	//	entry->write_proc = zyt_peripherals_proc_write;
	//}
}
/*删除/proc/zyt_peripherals文件*/
static void zyt_peripherals_remove_proc(void) {
	printk(KERN_ALERT"zyt_peripherals_remove_proc .\n");

	//remove_proc_entry(ZYT_PERIPHERALS_DEVICE_PROC_NAME, NULL);
	proc_remove(entry);
}
//----------------------------------------------------------------------------------
/*初始化设备*/
static int  __zyt_peripherals_setup_dev(struct zyt_peripherals_dev* dev) {
	int err;
	dev_t devno = MKDEV(zyt_peripherals_major, zyt_peripherals_minor);

	memset(dev, 0, sizeof(struct zyt_peripherals_dev));

	cdev_init(&(dev->dev), &zyt_peripherals_fops);
	dev->dev.owner = THIS_MODULE;
	dev->dev.ops = &zyt_peripherals_fops;        

	/*注册字符设备*/
	err = cdev_add(&(dev->dev),devno, 1);
	if(err) {
		return err;
	}        

	/*初始化信号量和寄存器val的值*/
	//init_MUTEX(&(dev->sem));
	dev->val = 0;

	return 0;
}

/*模块加载方法*/
static int __init zyt_peripherals_init(void){ 

	int err = -1;
	dev_t dev = 0;
	struct device* temp = NULL;

	printk(KERN_ALERT"Initializing zyt_peripherals device.\n");        

	/*动态分配主设备和从设备号*/
	err = alloc_chrdev_region(&dev, 0, 1, ZYT_PERIPHERALS_DEVICE_NODE_NAME);
	if(err < 0) {
		printk(KERN_ALERT"Failed to alloc char dev region.\n");
		goto fail;
	}

	zyt_peripherals_major = MAJOR(dev);
	zyt_peripherals_minor = MINOR(dev);        

	/*分配zyt_peripherals_dev 设备结构体变量*/
	peripherals_dev = kmalloc(sizeof(struct zyt_peripherals_dev), GFP_KERNEL);
	if(!peripherals_dev) {
		err = -ENOMEM;
		printk(KERN_ALERT"Failed to alloc peripherals_dev.\n");
		goto unregister;
	}        

	/*初始化设备*/
	err = __zyt_peripherals_setup_dev(peripherals_dev);
	if(err) {
		printk(KERN_ALERT"Failed to setup dev: %d.\n", err);
		goto cleanup;
	}        

	/*在/sys/class/目录下创建设备类别目录zyt_peripherals*/
	zyt_peripherals_class = class_create(THIS_MODULE, ZYT_PERIPHERALS_DEVICE_CLASS_NAME);
	if(IS_ERR(zyt_peripherals_class)) {
		err = PTR_ERR(zyt_peripherals_class);
		printk(KERN_ALERT"Failed to create zyt_peripherals class.\n");
		goto destroy_cdev;
	}        

	/*在/dev/目录和/sys/class/zyt_peripherals目录下分别创建设备文件zyt_peripherals*/
	temp = device_create(zyt_peripherals_class, NULL, dev, "%s", ZYT_PERIPHERALS_DEVICE_FILE_NAME);
	if(IS_ERR(temp)) {
		err = PTR_ERR(temp);
		printk(KERN_ALERT"Failed to create zyt_peripherals device.");
		goto destroy_class;
	}        

	/*在/sys/class/zyt_peripherals/zyt_peripherals目录下创建属性文件val*/
	err = device_create_file(temp, &dev_attr_val);
	if(err < 0) {
		printk(KERN_ALERT"Failed to create attribute val.");                
		goto destroy_device;
	}

	dev_set_drvdata(temp, peripherals_dev);        

	/*创建/proc/zyt_peripherals文件*/
	zyt_peripherals_create_proc();

	printk(KERN_ALERT"Succedded to initialize zyt_peripherals device.\n");
	return 0;

destroy_device:
	device_destroy(zyt_peripherals_class, dev);

destroy_class:
	class_destroy(zyt_peripherals_class);

destroy_cdev:
	cdev_del(&(peripherals_dev->dev));

cleanup:
	kfree(peripherals_dev);

unregister:
	unregister_chrdev_region(MKDEV(zyt_peripherals_major, zyt_peripherals_minor), 1);

fail:
	return err;
}

/*模块卸载方法*/
static void __exit zyt_peripherals_exit(void) {

	dev_t devno = MKDEV(zyt_peripherals_major, zyt_peripherals_minor);

	printk(KERN_ALERT"Destroy zyt_peripherals device.\n");        

	/*删除/proc/zyt_peripherals文件*/
	zyt_peripherals_remove_proc();        

	/*销毁设备类别和设备*/
	if(zyt_peripherals_class) {
		device_destroy(zyt_peripherals_class, MKDEV(zyt_peripherals_major, zyt_peripherals_minor));
		class_destroy(zyt_peripherals_class);
	}        

	/*删除字符设备和释放设备内存*/
	if(peripherals_dev) {
		cdev_del(&(peripherals_dev->dev));
		kfree(peripherals_dev);
	}        

	/*释放设备号*/
	unregister_chrdev_region(devno, 1);

}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ZYT Peripherals Driver");

module_init(zyt_peripherals_init);
module_exit(zyt_peripherals_exit);


