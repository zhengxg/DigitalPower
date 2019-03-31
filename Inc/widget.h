#ifndef __WIDGET_H__
#define __WIDGET_H__

typedef struct XG_POINT_S{
    int x;
    int y;
}xg_point_t;

typedef struct XG_RECT_S{
    xg_point_t start_point;
    xg_point_t stop_point;
}xg_rect_t;

typedef enum XG_WIDGET_TYPE_E{
    E_XG_WIDGET_TYPE_PANEL = 1,
    E_XG_WIDGET_TYPE_LABLE
}xg_widget_type_e;

typedef enum XG_ALIGN_VERTICAL_E{
    E_XG_ALIGN_VERTICAL_TOP = 1,
    E_XG_ALIGN_VERTICAL_CENTER = 2,
    E_XG_ALIGN_VERTICAL_BOTTON = 3
}xg_align_vertical_e;

typedef enum XG_ALIGN_HORIZONTAL_E{
    E_XG_ALIGN_HORIZONTAL_LEFT = 1,
    E_XG_ALIGN_HORIZONTAL_CENTER = 2,
    E_XG_ALIGN_HORIZONTAL_RIGHT = 3
}xg_align_horizontal_e;




typedef struct XG_WIDGET_S{
    xg_point_t start_point;
    xg_point_t size;
    xg_widget_type_e type;
    xg_widget_t* next_widget;
}xg_widget_t;

typedef xg_widget_t xg_widget_panel_t;


typedef struct XG_WIDGET_LABLE_S{
    xg_widget_t widget;
    char* text;
    char* hint;
    lv_font_t font;
    int font_spacing;
    xg_align_horizontal_e align_horizontal;
    xg_align_vertical_e align_vertical;
}xg_widget_lable_t;



#endif
