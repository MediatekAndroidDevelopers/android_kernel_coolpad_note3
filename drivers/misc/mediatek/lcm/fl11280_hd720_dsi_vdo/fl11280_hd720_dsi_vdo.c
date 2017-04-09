/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
#else
#include <linux/string.h>
#endif

#include "lcm_drv.h"

//#include <prj/prj_config.h>


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720) 
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_FL11280  (0x1128)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0xB9,3,{0xF1,0x12,0x80}},	

{0xBA,27,{0x32,0x81,0x05,0xF9,0x0E,0x0E,0x02,0x00,0x00,
                                     0x00,0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0A,
                                     0x00,0x00,0x02,0x4F,0x00,0x00,0x00,0x27}}, 
//0x33:4lANE,0x32:3LANE

{0xB8,1,{0x26}},

{0xB3,17,{0x02,0x00,0x06,0x06,0x07,0x0B,0x1E,0x1E,0x00,
                                     0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x00}}, 

{0xC0,9,{0x73,0x73,0x50,0x50,0x00,0x00,0x08,0x30,0x00}},

{0xBC,1,{0x46}}, 
{0xCC,1,{0x0B}}, 
{0xB4,1,{0x00}}, 
{0xB2,1,{0xC8}}, 


{0xE3,7,{0x03,0x03,0x03,0x03,0x03,0x03,0x00}}, 

{0xB1,9,{0x23,0x23,0x1E,0x1E,0x33,0x77,0x01,0x9B,0x0C}}, 

{0xB5,2,{0x0A,0x0A}}, 

{0xB6,2,{0x66,0x66}}, //6B 80
{0xE9,55,{0x00,0x00,0x05,0x00,0x00,0x08,0xB0,0x12,0x31,
                                    0x00,0x38,0x09,0x0A,0xB0,0x37,0x02,0x00,0x30,0x00,
                                    0x00,0x00,0x18,0x20,0x64,0x02,0x88,0x88,0x88,0x88,
                                    0x88,0x84,0x89,0x31,0x75,0x13,0x88,0x88,0x88,0x88,
                                    0x88,0x85,0x89,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                    0x00,0x00,0x00,0x00,0x00,0x00}}, 

{0xEA,42,{0x02,0x1A,0x00,0x00,0x57,0x13,0x31,0x88,0x88,
                                     0x88,0x88,0x88,0x85,0x98,0x46,0x02,0x20,0x88,0x88,
                                     0x88,0x88,0x88,0x84,0x98,0x00,0x00,0x00,0x00,0x00,
                                     0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                     0x00,0x00,0x00}}, 

{0xE0,34,{0x00,0x1C,0x21,0x3F,0x3F,0x3F,0x2C,0x46,0x09,
                                     0x0D,0x10,0x14,0x15,0x13,0x14,0x0F,0x1F,0x00,0x1C,
                                     0x21,0x3F,0x3F,0x3F,0x2C,0x46,0x09,0x0D,0x10,0x14,
                                     0x15,0x13,0x14,0x0F,0x1F}}, 
{0x11, 0,{0x00}},

{REGFLAG_DELAY, 100, {}},

{0x29, 0,{0x00}},
{REGFLAG_DELAY, 100, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {

	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_in_setting[] = {
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_read_id[] = {
	{0xFF,3,{0x98,0x81,0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	
	for(i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;
		
		switch (cmd) {	
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE :
			break;
		default:
			//if (cmd == 0x53)
			//	table[i].para_list[0] -= 2;
				
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 

	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_THREE_LANE;//LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 17;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 20;
	params->dsi.horizontal_backporch				= 80;
	params->dsi.horizontal_frontporch				= 80;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	params->dsi.PLL_CLOCK = 230;

	params->dsi.HS_TRAIL=15;

    params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 0;
	
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_in_setting, sizeof(lcm_deep_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

	
}

static void lcm_resume(void)
{
	lcm_init();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
	#if 0
	unsigned int id0,id1,id=0;
	unsigned char buffer[2];
	unsigned int array[16];  
	
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100);

	array[0]=0x00043902;
	array[1]=0x018198ff;// page enable
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);

	array[0]=0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x00, buffer, 1);
	id0  =  buffer[0]; 
	
	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01, buffer, 1);
	id1  =  buffer[0]; 
	id = (id0 << 8) | id1;
		


	return (LCM_ID_FL11280 == id)?1:0;
	#endif
}

LCM_DRIVER fl11280_hd720_dsi_vdo = 
{
	.name		= "fl11280_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
