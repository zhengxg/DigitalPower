#include "main.h"
#include "menu.h"
#include "UI/lv_font.h"
#include "UI/lv_conf.h"


#include "oled.h"

void menu_main_display(void)
{
    char TextBuffer[50];
    int voltage,current;
    voltage = g_SystemInfo.SetVoltage;
    current = g_SystemInfo.SetCurrent;
    sprintf(TextBuffer, "Set: %2d.%02dV %d.%02dA ", voltage/100, voltage%100, current/100, current%100);
    OledDrawText(&lv_font_yahei_16, 0, 0, TextBuffer);

    voltage = g_SystemInfo.RtVoltage;
    current = g_SystemInfo.RtCurrent;
    sprintf(TextBuffer, "%2d.%02dV %d.%02dA ", voltage/100, voltage%100, current/100, current%100);
    OledDrawText(&lv_font_yahei_24, 10, 18, TextBuffer);

    sprintf(TextBuffer, "2019-04-13 15:29:00");
    OledDrawText(&lv_font_yahei_16, 0, 44, TextBuffer);
    OledDisplayRefresh();
    DEBUG("Refresh menu main\r\n");
}

