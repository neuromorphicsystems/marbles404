/*************************************************************************
* SCAMP Vision Chip Development System Library
*------------------------------------------------------------------------
* Copyright (c) 2020 The University of Manchester. All Rights Reserved.
*
*************************************************************************/

#include <cstdlib>
#include "scamp5d_tcp.h"
#include "scamp5d_usb.h"
#include "scamp5d_proxy.h"
#include "vs_packet_decoder.hpp"
#include "debug_functions.h"


const char*box_name;
scamp5d_interface*box;
scamp5d_proxy *proxy;

scamp5d_interface*box_a;
scamp5d_interface*box_b;
scamp5d_proxy *proxy_a;
scamp5d_proxy *proxy_b;
vs_packet_decoder*packet_switch;

std::atomic<bool> quit(false);
bool host_on = false;
char filepath[256];
uint8_t bitmap24_buffer[256*256*3];


void setup_packet_switch(){
    
    // text are from the 'vs_post_text' function
    packet_switch->case_text([&](const char*text,size_t length){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: text, lc=%d: \n%s\n",
            box_name,sn,lc,text);
        //        // report back some user values
        //        box->post_message(VS_MSG_ROUTE_APP,VS_MSG_USER_VALUE,23,sn);
    });

    // boundingbox are from the 'scamp5_output_boundingbox' function
    packet_switch->case_boundingbox([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: aabb, lc=%d, { %d, %d, %d, %d }\n",
            box_name,sn,lc,data(0,0),data(0,1),data(1,0),data(1,1));
    });

    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_analog([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: aout, lc=%d, width=%d, height=%d\n",
            box_name,sn,lc,width,height);
    });

    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_digital([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: dout, lc=%d, width=%d, height=%d\n",
            box_name,sn,lc,width,height);
    });

    // ponits are data from the 'scamp5_output_events' function
    packet_switch->case_points([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: points, lc=%d, num_points=%d\n",
            box_name,sn,lc,data.get_row_size());
    });

    packet_switch->case_raw([&](const uint8_t*payload,size_t bytes){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("%s packet[%d]: raw, lc=%d, payload bytes=%d, [ %2.2X, %2.2X, %2.2X, %2.2X ... ]\n",
            box_name,sn,lc,bytes,payload[0],payload[1],payload[2],payload[3]);
    });

    packet_switch->case_data_int32([&](const vs_array<int32_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        auto channel = packet_switch->get_data_channel();
        printf("%s packet[%d]: data array, lc=%d, int32[%d][%d], channel: %d\n",
            box_name,sn,lc,data.get_row_size(),data.get_col_size(),channel);
    });

}


void input_loop(){

    printf("<press Q to quit>\n");

    while(quit==false){
        char c = conio_getch();
        switch(c){

        case 'q':
            quit = true;
            printf("quit\n");
            break;

        case 's':// for diagnostic purpose
            printf("signature_counter: %d, packet_counter: %d\n",
                box->get_signature_counter(),box->get_packet_counter());
            break;

        case 'h':// manually toggle whether the device should behave as if the host app running
            if(host_on){
                box_a->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_DC);
                box_b->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_DC);
                printf("host off\n");
            }else{
                box_a->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_ON);
                box_b->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_ON);
                printf("host on\n");
            }
            host_on = !host_on;
            break;

        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
            printf("move first user slider: %d\n",c - '0');
            box->post_message(VS_MSG_ROUTE_APP,VS_MSG_GUI_UPDATE,VS_GUI_USER_ITEM_0,c - '0');
            break;
        }
    }
}


int main(int argc,char*argv[]){
    char ip_str[20] = "192.168.1.101";
    int r;

    // use the IP address in argument if possible
    if(argc>=2){
        snprintf(ip_str,20,"%s",argv[1]);
    }

    printf("Scamp Vision Systen Dual USB Connection Example\n");

    // a thread to handle console keyboard input
    std::thread input_thread(input_loop);

    // 'scamp5d_packet_switch' the object to decode the incoming packets and pass them
    // to the corresponding handler function registered.
    packet_switch = new vs_packet_decoder;

    setup_packet_switch();


    /// Initialize the device interface

    box_a = new scamp5d_usb();
    r = box_a->open("",0);
    if(r){
        fprintf(stderr,"<Error> failed to open USB device!\n");
        exit(-1);
    }else{
        printf("<Box A Ready>\n------------------------------------------------\n");
    }

    box_b = new scamp5d_usb();
    r = box_b->open("",1);
    if(r){
        fprintf(stderr,"<Error> failed to open USB device!\n");
        exit(-1);
    }else{
        printf("<Box B Ready>\n------------------------------------------------\n");
    }

    /// Setup how the interface handles packet received from the device
    // Here the "free_packet" callback is used, which needs to 'free' the packet after use.
    auto free_packet_callback = [&](uint8_t*packet,size_t packet_size){

        // process the packet
        packet_switch->decode_packet(packet,packet_size,box->get_packet_counter());

        // forward to proxy, and deal with proxy bottle neck
        const size_t warning_size = 512*1024;
        const size_t discard_size = 10*1024*1024;
        size_t queue_size = proxy->get_bytes_in_queue();

        if(queue_size>discard_size){
            printf("[scamp5d_proxy] warning: queue over %d bytes, packet discarded!!!\n",queue_size);
            // dont broadcast, just free
            free(packet);
        }else{
            if(queue_size>warning_size){
                printf("[scamp5d_proxy] warning: over %d bytes of data queued!\n",queue_size);
            }
            // broadcast through the proxy and free the packet
            proxy->broadcast(packet,packet_size,true);
        }

    };

    box_a->on_free_packet(free_packet_callback);
    box_b->on_free_packet(free_packet_callback);


    /// Initialize the TCP Proxy

    proxy_a = new scamp5d_proxy;
    proxy_a->open(ip_str,27725);

    proxy_b = new scamp5d_proxy;
    proxy_b->open(ip_str,27726);

    /// Setup how the proxy handles packet received from its clients
    auto proxy_receive_callback = [&](const uint8_t*packet,size_t bytes){
        //printf("client -> device: %d, %s\n",bytes,HEX_STR(packet,(bytes>16)? 16:bytes));
        printf("client -> device: %d bytes\n",bytes);
        box->write(packet,bytes);
    };

    proxy_a->on_receive_packet(proxy_receive_callback);
    proxy_b->on_receive_packet(proxy_receive_callback);

    /// Main Loop
    while(quit==false){
        proxy = proxy_a;
        box = box_a;
        box_name = "box_a";
        box->routine();
        proxy->routine();

        proxy = proxy_b;
        box = box_b;
        box_name = "box_b";
        box->routine();
        proxy->routine();
    }

    proxy_a->close();
    delete proxy_a;

    proxy_b->close();
    delete proxy_b;

    box_a->close();
    delete box_a;

    box_b->close();
    delete box_b;

    delete packet_switch;

    input_thread.join();

    return 0;
}
