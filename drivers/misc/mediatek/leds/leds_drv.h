#include <linux/leds.h>
#include <leds_hal.h>
#if 1 //wangsl 
#include <linux/gpio.h>
//#define GPIO_LED_R_MODE_PIN
#define GPIO_LED_G_MODE_PIN  64
#define GPIO_LED_B_MODE_PIN  63
#endif
/****************************************************************************
 * LED DRV functions
 ***************************************************************************/

#ifdef CONTROL_BL_TEMPERATURE
int setMaxbrightness(int max_level, int enable);
#endif

extern int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness level);
#ifdef CONFIG_MTK_LEDS
extern int backlight_brightness_set(int level);
#else
#define backlight_brightness_set(level) do { } while (0)
#endif
extern int disp_bls_set_max_backlight(unsigned int level);
