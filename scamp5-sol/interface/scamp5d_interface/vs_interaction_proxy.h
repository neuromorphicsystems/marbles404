/*
* This module allows some of the 'vs_interaction' functions to be used by a interface application
* running on a computer. This can be used when graphic drawing or image display workloads are not 
* suiable for the device to run. 
*/
#pragma once

#include "scamp5d_proxy.h"
#include "vs_packet_decoder.hpp"
#include "vs_interaction_base.hpp"
#include "vs_interaction_graphics.hpp"

void vs_interaction_proxy_setup(vs_packet_decoder*device_packet_decoder,scamp5d_proxy*proxy);
void vs_interaction_proxy_clear();
