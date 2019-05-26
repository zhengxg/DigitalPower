#ifndef __MENU_H__
#define __MENU_H__


typedef enum{
    MENU_EVENT_100MS = 0,
    MENU_EVENT_BTN_SELECT,
    MENU_EVENT_BTN_UP,
    MENU_EVENT_BTN_DOWN,
    MENU_EVENT_BTN_START,
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

extern void menuInit(void);
extern void menuCallback(MenuEvnet_t event);

#endif
