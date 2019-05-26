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

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }
#define WIDTH     128
#define HEIGHT  64

#define BLACK   0
#define WHITE   1
#define INVERSE 2
#define SSD1306_LCDWIDTH    WIDTH

typedef enum{
    TEXT_SELECT_SHOW_NORMAL = 0,     //普通
    TEXT_SELECT_SHOW_WHITE,          //全白
    TEXT_SELECT_SHOW_BLACK,          //全黑
    TEXT_SELECT_SHOW_TRANSPARENT,    //透明
    TEXT_SELECT_SHOW_INVERT          //反转
}TextSelectShow_t;


extern uint8_t OledDisplayBuffer[];

extern void OLED_Init(void);
extern void OledDrawAll(uint16_t color);
extern int OledDrawText(lv_font_t * font_p, int xStart, int yStart, char* string);
extern int OledDrawTextWithSelect(lv_font_t * font_p, int xStart, int yStart, char* string, int selectStartIndex, int selectLen, TextSelectShow_t showtype);

extern void OledDisplayRefresh(void);



#endif
