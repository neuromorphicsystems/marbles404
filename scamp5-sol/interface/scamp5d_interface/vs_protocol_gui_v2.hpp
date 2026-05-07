/*************************************************************************
 * SCAMP Vision Chip Development System Library
 *------------------------------------------------------------------------
 * Copyright (c) 2020 The University of Manchester. All Rights Reserved.
 *
 *************************************************************************/

#ifndef VS_PROTOCOL_GUI_V2_HPP
#define VS_PROTOCOL_GUI_V2_HPP


#include "vs_protocol_gui.hpp"


#define VS_GUI_NULL    	0
#define VS_GUI_BUTTON  	1
#define VS_GUI_SLIDER  	2
#define VS_GUI_SWITCH	3
#define VS_GUI_CHECKBOX	4
#define VS_GUI_LIST		5

#define VS_GUI_IMAGE   	16
#define VS_GUI_SCOPE   	17
#define VS_GUI_BARPLOT 	18
#define VS_GUI_SCATTER 	19
#define VS_GUI_DISPLAY 		20
#define VS_GUI_FIGURE 		21
#define VS_GUI_LINEPLOT 	22


typedef uint32_t str_key_t;
typedef uint16_t draw_cmd_t;


class vs_gui_display_form{
public:
    str_key_t name;
    str_key_t style;
	int16_t type;
	int16_t size[2];
	int16_t pos[2];
	uint16_t options;
	vs_gui_display_form(){
		type = VS_GUI_DISPLAY;
		options = 0;
	}
};


class vs_gui_figure_form:public vs_gui_display_form{
public:
    int32_t scale_min;
    int32_t scale_max;
    int32_t time_window;
    int16_t draw_arg;
    vs_gui_figure_form(){
    	type = VS_GUI_FIGURE;
    }
    vs_gui_figure_form(const vs_gui_display_form*base){
    	if(base){
    		*((vs_gui_display_form*)this) = *base;
    	}
    	type = VS_GUI_FIGURE;
    }
};


class vs_gui_scope_form:public vs_gui_figure_form{
public:
	vs_gui_scope_form(){
		type = VS_GUI_SCOPE;
		draw_arg = 1;
	}
	vs_gui_scope_form(const vs_gui_display_form*base):
		vs_gui_figure_form(base)
	{
		type = VS_GUI_SCOPE;
		draw_arg = 1;
	}
};


class vs_gui_barplot_form:public vs_gui_figure_form{
public:
	vs_gui_barplot_form(){
		type = VS_GUI_BARPLOT;
		draw_arg = 4;
	}
	vs_gui_barplot_form(const vs_gui_display_form*base):
		vs_gui_figure_form(base)
	{
		type = VS_GUI_BARPLOT;
		draw_arg = 4;
	}
};


class vs_gui_lineplot_form:public vs_gui_figure_form{
public:
	vs_gui_lineplot_form(){
		type = VS_GUI_LINEPLOT;
		draw_arg = 1;
	}
	vs_gui_lineplot_form(const vs_gui_display_form*base):
		vs_gui_figure_form(base)
	{
		type = VS_GUI_LINEPLOT;
		draw_arg = 1;
	}
};


class vs_gui_scatter_form:public vs_gui_figure_form{
public:
	int32_t x_min;
	int32_t x_max;
	vs_gui_scatter_form(){
		type = VS_GUI_SCATTER;
		draw_arg = 1;
	}
	vs_gui_scatter_form(const vs_gui_display_form*base):
		vs_gui_figure_form(base)
	{
		type = VS_GUI_SCATTER;
		draw_arg = 1;
	}
};


//------------------------------------------------------------------------------


class vs_gui_element_form{

public:
    str_key_t name;
	int16_t type;
	uint16_t flag;

	vs_gui_element_form(){
		type = VS_GUI_NULL;
	}
};


class vs_gui_button_form:public vs_gui_element_form{

public:
	vs_gui_button_form(){
		type = VS_GUI_BUTTON;
	}
};


class vs_gui_switch_form:public vs_gui_element_form{

public:
	int32_t init_value;
	vs_gui_switch_form(){
		type = VS_GUI_SWITCH;
	}
};


class vs_gui_slider_form:public vs_gui_element_form{

public:
	int32_t min_value;
	int32_t max_value;
	int32_t init_value;
	int32_t step_value;
	vs_gui_slider_form(){
		type = VS_GUI_SLIDER;
	}
};


//------------------------------------------------------------------------------


#endif
