/*
 * vs_interaction_base.cpp
 *
 *  Created on: 8 May 2024
 *      Author: chen2
 */

#ifndef VS_INTERACTION_BASE_HPP
#define VS_INTERACTION_BASE_HPP

#include <vs_common.h>
#include <vs_protocol.hpp>
#include <vs_protocol_gui_v2.hpp>


class vs_gui_interaction_base{

protected:
	vs_protocol_out output_protocol;

public:
	bool is_connected;

	vs_gui_interaction_base();
	virtual ~vs_gui_interaction_base();

	virtual uint32_t get_loop_counter()=0;
	virtual vs_protocol_out* claim_output_protocol()=0;
	virtual int send(const void*data,size_t bytes)=0;

};


extern vs_gui_interaction_base*vs_gui_context;


#endif
