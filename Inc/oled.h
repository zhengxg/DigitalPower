#ifndef __OLED_H__
#define __OLED_H__

#define OLED_SCK_PIN        GPIO_PIN_13
#define OLED_SCK_PORT       GPIOB

#define OLED_SDA_PIN        GPIO_PIN_15
#define OLED_SDA_PORT       GPIOB

#define OLED_RST_PIN        GPIO_PIN_8
#define OLED_RST_PORT       GPIOA

#define OLED_DC_PIN         GPIO_PIN_14
#define OLED_DC_PORT        GPIOB

#define OLED_CS_PIN         GPIO_PIN_12
#define OLED_CS_PORT        GPIOB

extern uint8_t OledDisplayBuffer[];

extern void OLED_Init(void);
extern int OledDrawText(lv_font_t * font_p, int xStart, int yStart, char* string);
extern void OledDisplayRefresh(void);



#endif
