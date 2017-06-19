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

#include <linux/yl_lcd.h>

static long cmdline_lcd_id;

const char *get_panel_name(const char *panel_name)
{
	static const char *yl_panel_name;
	if (panel_name)
		yl_panel_name = panel_name;
	return yl_panel_name;
}

int get_lcd_type(char *options)
{
	if (!options) {
	    printk("lcd id should be added to cmdline\n");
		cmdline_lcd_id = -1;
		return 0;
	}
	printk("__phg_ options= %s \n",options);
	return kstrtol(options, 0, &cmdline_lcd_id);
}

static char *get_lcd_id2str(int id)
{
	switch (id) {
	case YL_OTM8018B_BOOYI_FWVGA_450_ID:
		return "LCD_TYPE_OTM8018B_BOOYI_FWVGA_450";
	case YL_OTM9605A_BOOYI_QHD_450_ID:
		return "LCD_TYPE_OTM9605A_BOOYI_QHD_450";
	case YL_OTM9605A_BO0YI_QHD_500_ID:
		return "LCD_TYPE_OTM9605A_BO0YI_QHD_500";
	case YL_OTM1283A_BOOYI_HD_600_ID:
		return "LCD_TYPE_OTM1283A_BOOYI_HD_600";
	case YL_OTM9605A_YASSY_QHD_450_ID:
		return "LCD_TYPE_OTM9605A_YASSY_QHD_450";
	case YL_OTM9605A_YASSY_QHD_500_ID:
		return "LCD_TYPE_OTM9605A_YASSY_QHD_500";
	case YL_HX8394A_TIANMA_HD_600_ID:
		return "LCD_TYPE_HX8394A_TIANMA_HD_600";
	/* The funtionality of reading ID hasn't been implemented */
	default:
		return "LCD_TYPE_OTM8018B_BOOYI_FWVGA_450";
	}
}

ssize_t lcd_read_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len;
	const char *lcd_name;

	lcd_name = get_panel_name(NULL);
	printk("__phg__ cmdline_lcd_id=%ld \n",cmdline_lcd_id);
	if (!lcd_name)
		lcd_name = get_lcd_id2str((int)cmdline_lcd_id);
	len = strlen(lcd_name) + 2; /* line break plus null */
	len = snprintf(buf, len, "%s\n", lcd_name);

	return len;
}
