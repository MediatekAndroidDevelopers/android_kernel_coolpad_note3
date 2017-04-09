#ifndef _ZYT_PERIPHERALS_H_
#define _ZYT_PERIPHERALS_H_

#include <linux/cdev.h>
#include <linux/semaphore.h>

#define USE_ZYT_PERIPHERALS

#define ZYT_PERIPHERALS_DEVICE_NODE_NAME  "zyt_peripherals"
#define ZYT_PERIPHERALS_DEVICE_FILE_NAME  "zyt_peripherals"
#define ZYT_PERIPHERALS_DEVICE_PROC_NAME  "zyt_peripherals"
#define ZYT_PERIPHERALS_DEVICE_CLASS_NAME "zyt_peripherals"

#define ZYT_PERIPHERALS_UNKNOWN	"Unknown"

//字符设备
struct zyt_peripherals_dev {
	int val;
	char peripherals_info[512];
	struct semaphore sem;
	struct cdev dev;
};

//外设类型枚举
typedef enum zyt_peripherals_type
{
	TYPE_TP = 0,
		
	TYPE_LCD,
	
	TYPE_CAMERA_MAIN,
	TYPE_CAMERA_SUB,
	
	TYPE_FALSH,
	TYPE_PLATFORM,
	
	TYPE_MAX
}E_ZYT_PERIPHERALS_TYPE;

//外设信息结构体类型
typedef struct peripherals_ID_2_name
{
	int id;
	char name[32];
}PERIPHERALS_ID_2_NAME;

//FLASH信息结构体类型
typedef struct flash_ID_2_name
{
	char id[32];
	char name[128];
}FLASH_ID_2_NAME;

//TP 外设信息
const PERIPHERALS_ID_2_NAME zyt_peripherals_TP_info[] = 
{	
	//GT-汇顶
	{0x0818, "GT868"},
	{0x0960, "GT960"},
	{0x0961,"GT960F"},
	{0x0813,"GT813"},
	{0x0814,"GT813TB"},
	{0x0819,"GT819"},
	{0x0927,"GT927"},
	{0x0928,"GT928"},
	{0x9110,"GT9110"},
	//MSG-辰明
	{0x2133, "MSG2133"},
	
	//IT-联阳
	{0x7260, "IT7260"},
	
	//FT-敦泰
	{0x5206, "FT5x06/FT6x06"},
	{0x5316, "FT5316"},
	{0x6206, "FT5x06/FT6x06"},

	//GSL-思立微
	{0x1680, "GSL1680"},

	//TMA-赛普拉斯
	{0x0140, "TMA140"},
	
	//ZET-君耀
	{0x6221, "ZET622X"},

	//
	{0x0168,"PIXCIR"},
	{0x0300, "Cy8ctma300"},
	{0x3203,"S3203"},
	{0x1896,"TM1896"},
	{0x7200,"PCAP7200"},
	{0x5208,"AW5208"},
	//
	//add your type here
	{0x11004,"NT11004"},

	//default
	{0x0000, ZYT_PERIPHERALS_UNKNOWN},
};

#endif
