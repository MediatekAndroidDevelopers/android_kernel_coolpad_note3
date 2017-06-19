/**********************************************************************
* yulong specific lcd id, include booyi, yashi, lead and so on
*
* Copyright (C) 2014  YULONG Company
*
* Copyright (C) 2014  Yu.wang
*              <yuwang@yulong.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
***********************************************************************/
#ifndef YL_LCD_H
#define YL_LCD_H

#include <linux/device.h>

enum PANEL_TYPE {
	PANEL_VENDOR_UNKNOWN = -1,

	/* Booyi Panel */
	YL_OTM8018B_BOOYI_FWVGA_450_ID = 0x10,
	YL_OTM9605A_BOOYI_QHD_450_ID,
	YL_OTM9605A_BO0YI_QHD_500_ID,
	YL_OTM1283A_BOOYI_HD_600_ID,

	/* Yassy Panel */
	YL_OTM9605A_YASSY_QHD_450_ID = 0x20,
	YL_OTM9605A_YASSY_QHD_500_ID,

	/* Tianma Panel*/
	YL_HX8394A_TIANMA_HD_600_ID = 0x30,
};

const char *get_panel_name(const char *panel_name);
int get_lcd_type(char *options);
ssize_t lcd_read_type(struct device *dev,
		struct device_attribute *attr, char *buf);

#endif /*YL_LCD_H*/
