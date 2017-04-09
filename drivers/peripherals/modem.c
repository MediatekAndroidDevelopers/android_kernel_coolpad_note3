#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/kernel.h>
//#include <linux/stdlib.h>
#include <linux/string.h>
#include <linux/dirent.h>
#include <linux/fcntl.h>
//#include <linux/pthread.h>
#include <linux/mount.h>
#include <linux/statfs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/termios.h> /* POSIX terminal control definitions */


//#include "common.h"
//#include "miniui.h"
//#include "ftm.h"
//#include "utils.h"

//#include "hardware/ccci_intf.h"

#define MD1_DUAL_CCCI_DEVICE_NODE "/dev/ttyC0"
#define MD2_DUAL_CCCI_DEVICE_NODE "/dev/ccci2_tty0"
#define MD5_EEMCS_DEVICE_NODE "/dev/eemcs_muxrp"

#define QUERY_SUPPORT_MODEM_COMMAND "AT+EPBSE=?\r\n"
#define QUERY_SUPPORT_MODEM_RESPOND "+EPBSE"

#define BUF_SIZE 128
#define HALT_INTERVAL 500

static struct file *file_atmd = NULL;
static struct file *file_atmd2 = NULL;
static struct file *file_atmd5 = NULL;

//static pthread_mutex_t ccci_mutex = PTHREAD_MUTEX_INITIALIZER;

static ssize_t send_at (struct file *file_atmd, const char *pCMD)
{
    ssize_t ret = 0, sent_len = 0;

	if (NULL == file_atmd) return -1;
	
    while (sent_len != strlen(pCMD))
    {
		if (!(file_atmd->f_op->write)) 
		{
			printk(KERN_ERR "zhuyifeng NO write function!\n"); 

			return -EIO;
		}


		ret = file_atmd->f_op->write(file_atmd, pCMD, strlen(pCMD), sent_len);

		//if (pthread_mutex_unlock (&ccci_mutex))
		//{
		//	printk(KERN_ERR "send_at pthread_mutex_unlock ERROR!\n"); 
		//}
		
        if (ret < 0)
        {
 			printk(KERN_ERR "zhuyifeng send_at write fail: fail code = %x\n", ret); 

            return ret;
        }
        else
        {	
            sent_len += ret;
 			//printk(KERN_ERR "[send_at] lenth = %d\n", sent_len); 
        }
    }

    return 0;		
}

static ssize_t wait4_ack (struct file *file_atmd, char *pACK, int timeout)
{
    char buf[BUF_SIZE] = {0};
	char *  p = NULL;
    int rdCount = 0, LOOP_MAX;
    ssize_t ret = -1;

    LOOP_MAX = timeout*1000/HALT_INTERVAL;

	if (NULL == file_atmd) return -1;

    printk(KERN_ERR "zhuyifeng Wait for AT ACK...: %s; Special Pattern: %s\n", buf, (pACK==NULL)?"NULL":pACK);

    for(rdCount = 0; rdCount < LOOP_MAX; ++rdCount) 
    {
		memset(buf,'\0',BUF_SIZE);
		   
		
		ret = file_atmd->f_op->read(file_atmd, buf, BUF_SIZE, 0);
		
        printk(KERN_ERR "zhuyifeng AT CMD ACK: %s.rdCount=%d\n", buf,rdCount);
        p = NULL;

		if (pACK != NULL)  
        {
	          p = strstr(buf, pACK);
			  
              if(p) 
			  {
                  ret = 0; break; 
              }
			  
			  p = strstr(buf, "ERROR");
			  
        	  if(p) 
			  {
                  ret = -1; break;
        	  }
			  
			  p = strstr(buf, "NO CARRIER");
        	  
			  if(p)
			  {
                  ret = -1; break;
        	  }
        }
		else
		{
			p = strstr(buf, "OK");
			
        	if(p) 
			{
	            printk(KERN_ERR "zhuyifeng Char before OK are %c,%c.\n",*(p-2),*(p-1) );			
	            if(*(p-2) == 'E' && *(p-1) == 'P')
				{
	                printk(KERN_ERR "zhuyifeng EPOK detected\n");			
	            }
				else
				{	
	                printk(KERN_ERR "zhuyifeng OK response detected\n");			            
	            	ret = 0; break;
	        	}
	        }
			
        	p = strstr(buf, "ERROR");
        	
			if(p) 
			{
            	ret = -1; break;
        	}
			
			p = strstr(buf, "NO CARRIER");
			
        	if(p) 
			{
            	ret = -1; break;
        	}
		}
	
		schedule();
//        udelay(HALT_INTERVAL);
    }

	printk(KERN_ERR "zhuyifeng ret = %d",ret);

    return ret;
}

void modem_get_band_string(void)
{

printk(KERN_ERR "enter zhuyifeng modem_get_band_string\n");


	file_atmd = filp_open("/dev/ttyC0", O_RDWR | O_NONBLOCK, 0);
	file_atmd2 = filp_open("/dev/ccci2_tty0", O_RDWR | O_NONBLOCK, 0);
	file_atmd5 = filp_open(MD5_EEMCS_DEVICE_NODE, O_RDWR | O_NONBLOCK, 0);

	if (IS_ERR(file_atmd))
	{
		file_atmd = NULL;

		printk(KERN_ERR "zhuyifeng file_atmd open fail\n");
	}
	else
	{
		printk(KERN_ERR "zhuyifeng file_atmd open ok\n");
	}


	if (IS_ERR(file_atmd2))
	{
		file_atmd = NULL;

		printk(KERN_ERR "zhuyifeng file_atmd2 open fail\n");
	}
	else
	{
		printk(KERN_ERR "zhuyifeng file_atmd2 open ok\n");
	}



	
	if (IS_ERR(file_atmd5))
	{
		file_atmd = NULL;

		printk(KERN_ERR "zhuyifeng file_atmd5 open fail\n");
	}
	else
	{
		printk(KERN_ERR "zhuyifeng file_atmd5 open ok\n");
	}

#if 0	
	printk(KERN_ERR "enter zhuyifeng modem_get_band_string\n");

	if (0 == send_at(file_atmd, QUERY_SUPPORT_MODEM_COMMAND))
	{
		if (0 == wait4_ack(file_atmd, QUERY_SUPPORT_MODEM_RESPOND, 100))
		{
			printk(KERN_ERR "zhuyifeng OK: send_at fd_atmd\n");
			
			//Do something
		}
		else
		{
			printk(KERN_ERR "zhuyifeng ERROR: wait4_ack fd_atmd\n");
		}
	}
	else
	{
		printk(KERN_ERR "zhuyifeng  ERROR: send_at QUERY_SUPPORT_MODEM_COMMAND fd_atmd\n");
	}
	

	if (0 == send_at(file_atmd2, QUERY_SUPPORT_MODEM_COMMAND))
	{
		if (0 == wait4_ack(file_atmd2, QUERY_SUPPORT_MODEM_RESPOND, 100))
		{
			printk(KERN_ERR "zhuyifeng OK: send_at fd_atmd2\n");
			
			//Do something
		}
		else
		{
			printk(KERN_ERR "zhuyifeng ERROR: wait4_ack fd_atmd2\n");
		}
	}
	else
	{
		printk(KERN_ERR "zhuyifeng ERROR: send_at QUERY_SUPPORT_MODEM_COMMAND fd_atmd2\n");
	}


	if (0 == send_at(file_atmd5, QUERY_SUPPORT_MODEM_COMMAND))
	{
		if (0 == wait4_ack(file_atmd5, QUERY_SUPPORT_MODEM_RESPOND, 100))
		{
			printk(KERN_ERR "zhuyifeng OK: send_at fd_atmd5\n");
			
			//Do something
		}
		else
		{
			printk(KERN_ERR "zhuyifeng ERROR: wait4_ack fd_atmd5\n");
		}
	}
	else
	{
		printk(KERN_ERR "zhuyifeng ERROR: send_at QUERY_SUPPORT_MODEM_COMMAND fd_atmd5\n");
	}	
#endif

	if (file_atmd) 	filp_close(file_atmd, NULL);
	if (file_atmd2) filp_close(file_atmd2, NULL);
	if (file_atmd5) filp_close(file_atmd5, NULL);
	
	printk(KERN_ERR "leave zhuyifeng modem_get_band_string\n");

}

EXPORT_SYMBOL_GPL(modem_get_band_string);
