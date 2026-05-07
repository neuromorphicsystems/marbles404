/*************************************************************************
 * SCAMP Vision Chip Development System Library
 *------------------------------------------------------------------------
 * Copyright (c) 2020 The University of Manchester. All Rights Reserved.
 *
 *************************************************************************/

/*!

\file 

\ingroup VS_M0_CORE_MODULE

\author Jianing Chen

*/

#ifndef VS_INTERACTION_GRAPHICS_HPP
#define VS_INTERACTION_GRAPHICS_HPP

#include <array>
#include <functional>
#include <vs_common.h>
#include <vs_protocol_gui.hpp>


/*!
    @brief display an image given

    @param display    the display handle
    @param buffer     pixel buffer
    @param w    	  width
    @param h    	  height
    @param format     pixel format
*/
void vs_gui_display_image(vs_handle display,const uint8_t*buffer,uint8_t w,uint8_t h,vs_gui_imageformat_t format={1});


/*!
    @brief place a floating text on a display

    @param display    the display handle
    @param c    	column position
    @param r    	row position
    @param str    	the text string
    @param color    RGBA color of the text
*/
void vs_gui_display_text(vs_handle display,int16_t c,int16_t r,const char*str,std::array<uint8_t,4> color={255,255,255,255});


/*!
    @brief draw graphics over a display using a OpenGL-like API.

    @param display  the display handle
    @param f    	the callback function containing the drawing code
*/
void vs_gui_display_graphics(vs_handle display,std::function<void(void)> f);


/*!
This namespace contains functions that draw graphics in a host display. These fucntions
should only be used inside the callback function of ::vs_gui_display_graphics.
*/
namespace vs_gui_graphics_api{

/*!
    @brief push a copy of the current transformation matrix into a stack

    Note: the graphics functions are similar to OpenGL, more details by
    reading OpenGL related documentation.
*/
void m_push();

/*!
    @brief pop a previous transformation matrix from a stack to replace the current one

    Note: the graphics functions are similar to OpenGL, more details by
    reading OpenGL related documentation.
*/
void m_pop();

/*!
    @brief right multiply a translation matrix to the transformation matrix

    @param x        x axis distance
    @param y    	y axis distance
*/
void m_translate(int16_t x,int16_t y);

/*!
    @brief right multiply a rotation matrix to the transformation matrix

    @param deg      angle to rotate in degree
*/
void m_rotate(int16_t deg);

/*!
    @brief right multiply a rotation matrix using 'atan2' function to describe the angle

    @param y      y length of 'atan2' function
    @param x      x length of 'atan2' function
*/
void m_rotate_atan2(int16_t y,int16_t x);

/*!
    @brief right multiply a scaling matrix to the transformation matrix

    @param x      numerator of the scaling factor for x
    @param y      numerator of the scaling factor for y
    @param den    denominator of the x and y scaling

    Final scaling factors are x/den and y/den.

    Example Usage:
    \code
    // scale x and y uniformly by a factor of 1.5:
    m_scale(150,150,100);
    \endcode
*/
void m_scale(int16_t x,int16_t y,int16_t den);

/*!
    @brief right multiply a scaling matrix to the transformation matrix using floating point numbers

    @param x      floating point x scaling factor
    @param y      floating point y scaling factor
*/
void m_scale(float x,float y);

/*!
    @brief change the color used when drawing

    @param rgba 	a array of byte specifying RGBA component

    Example Usage:
    \code
    // set color to opaque red
    set_color({255,0,0,255});
    \endcode
*/
void set_color(const std::array<uint8_t,4>&rgba);
void set_color(uint8_t r,uint8_t g,uint8_t b,uint8_t a=255);

/*!
    @brief change line width used when drawing

    @param w 	numerator of the line width
    @param den 	denominator of the line width
*/
void set_linewidth(int16_t w,int16_t den=1);

/*!
    @brief change the size of any points being drawn

    @param w 	numerator of the point size
    @param den 	denominator of the point size
*/
void set_pointsize(int16_t w,int16_t den=1);

/*!
    @brief draw a point

*/
void draw_point(int16_t x0,int16_t y0);

/*!
    @brief draw a line

*/
void draw_line(int16_t x0,int16_t y0,int16_t x1,int16_t y1);

/*!
    @brief draw a rectangular given the coordinates of its two diagonal vertices

*/
void draw_rect(int16_t x0,int16_t y0,int16_t x1,int16_t y1);

/*!
    @brief draw a circle given its centroid and radius

*/
void draw_circle(int16_t x0,int16_t y0,int16_t radius,int16_t n_segments=24);

};

#endif
