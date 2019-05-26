#ifndef __MENU_H__
#define __MENU_H__


typedef enum{
    MENU_EVENT_100MS = 0,
    MENU_EVENT_BTN_SELECT,
    MENU_EVENT_BTN_UP,
    MENU_EVENT_BTN_DOWN,
    MENU_EVENT_BTN_START
}MenuEvnet_t;


typedef void (*menuRefreshFunc)(void);
typedef void (*menuEventCallbackFunc)(MenuEvnet_t);

typedef struct{
    menuRefreshFunc  RefreshFunc;
    menuEventCallbackFunc   EventCallbackFunc;
    int blinkInterval;
    int systemTime; 
}MenuParam_t;

typedef struct{
    char text[20];
}MenuList_t;


typedef enum{
    MENU_SELECT_FUNC_SET_VOLTAGE = 0,
    MENU_SELECT_FUNC_SET_CURRENT,
    MENU_SELECT_FUNC_SET_TIME,
    MENU_SELECT_FUNC_SAVE_USR_CFG,
    MENU_SELECT_FUNC_LOAD_USR_CFG,
    MENU_SELECT_FUNC_LOAD_FACT_CFG,
    MENU_SELECT_FUNC_EXIT
}MenuSelectFunc_t;

extern void menuInit(void);
extern void menuCallback(MenuEvnet_t event);

#endif
