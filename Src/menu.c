#include <string.h>
#include <math.h>
#include "main.h"
#include "menu.h"
#include "UI/lv_font.h"
#include "UI/lv_conf.h"


#include "oled.h"

MenuParam_t gMenuParam;

MenuList_t SelectFuncList[] ={
    {"Set voltage"},
    {"Set current"},
    {"Set time"},
    {"Save usr cfg"},
    {"Load usr cfg"},
    {"Load fact cfg"},
    {"Exit"}
};

int menuSelectFunc_selectItem = 0;
int menuSelectFunc_lastSelectItem = 0;
int menuSetVoltage_volatge;
int menuSetVoltage_selectIndex = 0;


void menuMainRefresh(void);
void menuMainEventCallback(MenuEvnet_t event);
void menuSelectFuncRefresh(void);
void menuSelectFuncEventCallback(MenuEvnet_t event);
void menuSetVoltageRefresh(void);
void menuSetVoltageEventCallback(MenuEvnet_t event);

void menuCallback(MenuEvnet_t event)
{
    switch(event){
        case MENU_EVENT_100MS:
        {
            gMenuParam.systemTime+=100;
            if((gMenuParam.systemTime%gMenuParam.blinkInterval) == 0){
                if(gMenuParam.RefreshFunc)
                    gMenuParam.RefreshFunc();
            }
        }
        break;
        case MENU_EVENT_BTN_SELECT:
        case MENU_EVENT_BTN_UP:
        case MENU_EVENT_BTN_DOWN:
        case MENU_EVENT_BTN_START:
        {
            if(gMenuParam.EventCallbackFunc)
                gMenuParam.EventCallbackFunc(event);
        }
        break;
        default:
        break;
    }
}


void menuInit(void)
{
    gMenuParam.RefreshFunc = menuMainRefresh;
    gMenuParam.EventCallbackFunc = menuMainEventCallback;
    gMenuParam.systemTime = 0;
    gMenuParam.blinkInterval = 500;

    gMenuParam.RefreshFunc();
}

void menuMainRefresh(void)
{
    char TextBuffer[50];
    int voltage,current;

    OledDrawAll(BLACK);

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
    // DEBUG("Refresh menu Main\r\n");
}

void menuMainEventCallback(MenuEvnet_t event)
{
    if(event == MENU_EVENT_BTN_SELECT)
    {
        gMenuParam.RefreshFunc = menuSelectFuncRefresh;
        gMenuParam.EventCallbackFunc = menuSelectFuncEventCallback;
        DEBUG("switch to menu select func\r\n");
    }
}


void menuSelectFuncRefresh(void)
{
    char TextBuffer[50];
    TextSelectShow_t showtype;
    int yStart;
    int yEnd;
    static int startShowItem = 0;
    int i=0;

    OledDrawAll(BLACK);

    while(1){
        if(menuSelectFunc_selectItem < startShowItem)
            startShowItem = menuSelectFunc_selectItem;
        yStart = lv_font_yahei_16.h_px+ (menuSelectFunc_selectItem-startShowItem)*(lv_font_yahei_16.h_px);
        yEnd = yStart+lv_font_yahei_16.h_px;
        if(menuSelectFunc_selectItem > menuSelectFunc_lastSelectItem)
        {
            if(yEnd>HEIGHT)
            {
                startShowItem++;
                DEBUG("change start show item\r\n");
                continue;
            }
            else
                break;            
        }
        break;
    }

    sprintf(TextBuffer, "Select function");
    OledDrawText(&lv_font_yahei_16, 0, 0, TextBuffer);
    yStart = lv_font_yahei_16.h_px;

    for(i=0;i<(sizeof(SelectFuncList)/sizeof(MenuList_t));i++)
    {
        if(i >= startShowItem)
        {
            showtype = (menuSelectFunc_selectItem == i)?TEXT_SELECT_SHOW_INVERT:TEXT_SELECT_SHOW_NORMAL;
            sprintf(TextBuffer, SelectFuncList[i].text);
            OledDrawTextWithSelect(&lv_font_yahei_16, 16, yStart, TextBuffer, 0, strlen(TextBuffer), showtype);
            yStart += lv_font_yahei_16.h_px;
        }
    }
    
    OledDisplayRefresh();
    // DEBUG("Refresh menu SelectFunc\r\n");
}


void menuSelectFuncEventCallback(MenuEvnet_t event)
{
    switch(event){
        case MENU_EVENT_BTN_SELECT:
        {
            DEBUG("Select item %d %s\r\n", menuSelectFunc_selectItem, \
            SelectFuncList[menuSelectFunc_selectItem].text);
            if(menuSelectFunc_selectItem == MENU_SELECT_FUNC_SET_VOLTAGE)
            {
                gMenuParam.RefreshFunc = menuSetVoltageRefresh;
                gMenuParam.EventCallbackFunc = menuSetVoltageEventCallback;
                menuSetVoltage_volatge = g_SystemInfo.SetVoltage;
                gMenuParam.RefreshFunc();
            }
            else if(menuSelectFunc_selectItem == MENU_SELECT_FUNC_EXIT)
            {
                gMenuParam.RefreshFunc = menuMainRefresh;
                gMenuParam.EventCallbackFunc = menuMainEventCallback;
                gMenuParam.RefreshFunc();
            }
        }
        break;
        case MENU_EVENT_BTN_UP:
        {
            menuSelectFunc_lastSelectItem = menuSelectFunc_selectItem;
            if(menuSelectFunc_selectItem > 0)
                menuSelectFunc_selectItem--;
            else
                menuSelectFunc_selectItem = sizeof(SelectFuncList)/sizeof(MenuList_t)-1;
        }
        break;
        case MENU_EVENT_BTN_DOWN:
        {
            menuSelectFunc_lastSelectItem = menuSelectFunc_selectItem;
            menuSelectFunc_selectItem++;
            menuSelectFunc_selectItem %= sizeof(SelectFuncList)/sizeof(MenuList_t);
        }
        break;

        default:
        break;
    }
    gMenuParam.RefreshFunc();
}


void menuSetVoltageRefresh(void)
{
    char TextBuffer[50];
    int blinkStatus;

    OledDrawAll(BLACK);

    sprintf(TextBuffer, "%s", SelectFuncList[MENU_SELECT_FUNC_SET_VOLTAGE].text);
    OledDrawText(&lv_font_yahei_16, 0, 0, TextBuffer);

    blinkStatus = (gMenuParam.systemTime/(gMenuParam.blinkInterval))%2;

    sprintf(TextBuffer, "%2d.%02dV", menuSetVoltage_volatge/100, menuSetVoltage_volatge%100);
    if((menuSetVoltage_selectIndex<2)&&(blinkStatus == 1))
        OledDrawTextWithSelect(&lv_font_yahei_24, 35, 18, TextBuffer,menuSetVoltage_selectIndex, 1, TEXT_SELECT_SHOW_INVERT);
    else if((menuSetVoltage_selectIndex<4)&&(blinkStatus == 1))
        OledDrawTextWithSelect(&lv_font_yahei_24, 35, 18, TextBuffer,menuSetVoltage_selectIndex+1, 1, TEXT_SELECT_SHOW_INVERT);
    else
        OledDrawTextWithSelect(&lv_font_yahei_24, 35, 18, TextBuffer,menuSetVoltage_selectIndex+1, 1, TEXT_SELECT_SHOW_NORMAL);
    

    sprintf(TextBuffer, "YES");
    if((menuSetVoltage_selectIndex==4)&&(blinkStatus == 1))
        OledDrawTextWithSelect(&lv_font_yahei_16, 0, HEIGHT-1-lv_font_yahei_16.h_px, TextBuffer, 0, strlen(TextBuffer), TEXT_SELECT_SHOW_INVERT);
    else
        OledDrawText(&lv_font_yahei_16, 0, HEIGHT-1-lv_font_yahei_16.h_px, TextBuffer);
    
    sprintf(TextBuffer, "NO");
    if((menuSetVoltage_selectIndex==5)&&(blinkStatus == 1))
        OledDrawTextWithSelect(&lv_font_yahei_16, WIDTH-1-strlen(TextBuffer)*lv_font_yahei_16.h_px, HEIGHT-1-lv_font_yahei_16.h_px, TextBuffer, 0, strlen(TextBuffer), TEXT_SELECT_SHOW_INVERT);
    else
        OledDrawText(&lv_font_yahei_16, WIDTH-1-strlen(TextBuffer)*lv_font_yahei_16.h_px, HEIGHT-1-lv_font_yahei_16.h_px, TextBuffer);

    OledDisplayRefresh();
    // DEBUG("Refresh menu Main\r\n");
}

void menuSetVoltageEventCallback(MenuEvnet_t event)
{
    if(event == MENU_EVENT_BTN_SELECT)
    {
        DEBUG("select index %d\r\n", menuSetVoltage_selectIndex);
        if(menuSetVoltage_selectIndex<4)
        {
            menuSetVoltage_selectIndex++;
            menuSetVoltage_selectIndex%=6;
        }
        else if(menuSetVoltage_selectIndex == 4)
        {
            g_SystemInfo.SetVoltage = menuSetVoltage_volatge;
            gMenuParam.RefreshFunc = menuSelectFuncRefresh;
            gMenuParam.EventCallbackFunc = menuSelectFuncEventCallback;
            DEBUG("switch to menu select func\r\n");
        }
        else if(menuSetVoltage_selectIndex == 5)
        {
            gMenuParam.RefreshFunc = menuSelectFuncRefresh;
            gMenuParam.EventCallbackFunc = menuSelectFuncEventCallback;
            DEBUG("switch to menu select func\r\n");
        }
        
    }
    else if(event == MENU_EVENT_BTN_UP)
    {
        if(menuSetVoltage_selectIndex<4)
        {
            if((menuSetVoltage_volatge+pow(10, 3-menuSetVoltage_selectIndex)) < g_SystemInfo.VoltageMax)
                menuSetVoltage_volatge += pow(10, 3-menuSetVoltage_selectIndex);
        }
        else
        {
            menuSetVoltage_selectIndex++;
            menuSetVoltage_selectIndex%=6;
        }
    }
    else if(event == MENU_EVENT_BTN_DOWN)
    {
        if(menuSetVoltage_selectIndex<4)
        {
            if((menuSetVoltage_volatge-pow(10, 3-menuSetVoltage_selectIndex))>g_SystemInfo.VoltageMin)
                menuSetVoltage_volatge-=pow(10, 3-menuSetVoltage_selectIndex);
        }
        else
        {
            if(menuSetVoltage_selectIndex>0)
                menuSetVoltage_selectIndex--;
            else
                menuSetVoltage_selectIndex = 5;
        }  
    }
    gMenuParam.RefreshFunc();
}