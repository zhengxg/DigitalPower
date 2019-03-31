#include "UI/lv_font.h"
#include "UI/lv_conf.h"

#include "widget.h"


void* xg_widget_new(xg_widget_type_e type)
{
    xg_widget_t* p_widget;
    if(type == E_XG_WIDGET_TYPE_PANEL)
    {
        p_widget = malloc(sizeof(xg_widget_panel_t));
        memset(p_widget, 0, sizeof(xg_widget_panel_t));
        p_widget->type = type;
    }
    else if(type == E_XG_WIDGET_TYPE_LABLE)
    {
        p_widget = malloc(sizeof(xg_widget_lable_t));
        memset(p_widget, 0, sizeof(xg_widget_lable_t));
        p_widget->type = type;
    }
    return p_widget;
}

xg_widget_t* xg_widget_init(xg_widget_t* p_widget, int start_x, int start_y, int size_x, int size_y)
{
    if(p_widget){
        p_widget->start_point.x = start_x;
        p_widget->start_point.y = start_y;
        p_widget->size.x = size_x;
        p_widget->size.y = size_y;
    }
    return p_widget;
}

int xg_widget_lable_add(xg_widget_t* parent_widget, xg_widget_t* widget){
    xg_widget_t* p_widget = parent_widget;
    while(p_widget->next_widget)
        p_widget = p_widget->next_widget;
    p_widget->next_widget = widget;
    return true;
}


xg_widget_lable_t* xg_widget_lable_init(xg_widget_lable_t* p_widget, int start_x, int start_y, char* text, lv_font_t font)
{
    xg_widget_init((xg_widget_t*)p_widget, start_x, start_y, 0, 0);

}


