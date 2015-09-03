#ifndef __UI_SYNC_LED_H__
#define __UI_SYNC_LED_H__

//Functions
void ui_sync_led_init(INT_VOID_F timestamp_in);
void ui_sync_led_on(void);
void ui_sync_led_off(void);
void ui_sync_led_flash(void);
void ui_sync_led_flash_pattern(void);
void ui_sync_led_set(unsigned int color, unsigned int pwm);

#endif /* __UI_SYNC_LED_H__ */
