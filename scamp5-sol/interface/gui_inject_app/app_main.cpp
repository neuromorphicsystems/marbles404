/*************************************************************************
* SCAMP Vision Chip Development System Library
*------------------------------------------------------------------------
* Copyright (c) 2020 The University of Manchester. All Rights Reserved.
*
*************************************************************************/

/*
* When the graphics drawing work is very complex, it can become a burden for 
* the MCU on the vision system. This application shows how to off-load the drawing
* code to the computer. To summarize, this application recive data from the box
* as usual. But when the data are relayed to the Host App via the proxy, extra
* packets containing the graphics for visualization are generated and injected into
* the data stream towards the Host App.
*/
#include <cstdlib>
#include "scamp5d_tcp.h"
#include "scamp5d_usb.h"
#include "scamp5d_proxy.h"
#include "vs_packet_decoder.hpp"
#include "vs_interaction_proxy.h"
#include "debug_functions.h"


scamp5d_proxy *proxy;
scamp5d_interface*box;
vs_packet_decoder*packet_switch;

std::atomic<bool> quit(false);
bool host_on = false;
char filepath[256];
uint8_t bitmap24_buffer[256*256*3];

uint32_t box_lc = 0;
vs_handle target_display;
bool packet_to_discard;


/*
* This function contiains the graphics drawing algorithm. The algorithm
* draws an arrow pointed towards the center of the image for each of
* the events received. It can be demonstrated by running the corner detection
* algorithm on the scamp box.
*/
void inject_graphics(vs_handle display,const vs_array<uint8_t>&data){
    printf("injecting graphics\n");
    vs_gui_display_graphics(display,[&](){
        using namespace vs_gui_graphics_api;
        const int display_scale = 2;
        const bool use_kernel_space = true;

        m_push();
        m_scale(display_scale*100,display_scale*100,100);
        if(use_kernel_space){
            m_push();
            m_translate(128,128);
            m_rotate(180);
            m_translate(-128,-128);
        }

        set_pointsize(30,10);// 10 is the denominator, so this means 3.0
        set_linewidth(20,10);

        // --------

        auto draw_arrow = [](){
            draw_line(-6,0,6,0);
            draw_line(0,3,6,0);
            draw_line(0,-3,6,0);
        };

        set_color({0,255,0,255});
        
        for(int i=0;i<data.get_row_size();i++){
            int ox = data(i,0);
            int oy = data(i,1);
            m_push();
            m_translate(ox,oy);
            m_rotate_atan2(oy-128,ox-128);
            draw_arrow();
            m_pop();
        }

        // --------

        if(use_kernel_space){
            m_pop();
        }
        m_pop();
    });
}


void setup_packet_switch(){

    // text are from the 'vs_post_text' function
    packet_switch->case_text([&](const char*text,size_t length){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: text, lc=%d: \n%s\n",sn,lc,text);
        //        // report back some user values
        //        box->post_message(VS_MSG_ROUTE_APP,VS_MSG_USER_VALUE,23,sn);
        box_lc = lc;
    });

    // boundingbox are from the 'scamp5_output_boundingbox' function
    packet_switch->case_boundingbox([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: aabb, lc=%d, { %d, %d, %d, %d }\n",sn,lc,data(0,0),data(0,1),data(1,0),data(1,1));
        box_lc = lc;
    });

#if 0
    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_analog([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: aout, lc=%d, width=%d, height=%d\n",sn,lc,width,height);
        target_display = packet_switch->get_display_handle();
        box_lc = lc;
    });

    // data from the 'scamp5_output_analog' and the 'scamp5_output_digital' function
    packet_switch->case_digital([&](int width,int height,uint8_t*bitmap_buffer){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: dout, lc=%d, width=%d, height=%d\n",sn,lc,width,height);
        box_lc = lc;
    });
#endif

    // ponits are data from the 'scamp5_output_events' function
    packet_switch->case_points([&](const vs_array<uint8_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: points, lc=%d, num_points=%d\n",sn,lc,data.get_row_size());
        box_lc = lc;

        // use these data to inject graphics to the host app display
        target_display = packet_switch->get_display_handle();
        inject_graphics(target_display,data);

        packet_to_discard = true;
    });

    packet_switch->case_raw([&](const uint8_t*payload,size_t bytes){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        printf("packet[%d]: raw, lc=%d, payload bytes=%d, [ %2.2X, %2.2X, %2.2X, %2.2X ... ]\n",sn,lc,bytes,
            payload[0],payload[1],payload[2],payload[3]);
        box_lc = lc;
    });

    packet_switch->case_data_int32([&](const vs_array<int32_t>&data){
        auto sn = packet_switch->get_packet_sn();
        auto lc = packet_switch->get_loop_counter();
        auto channel = packet_switch->get_data_channel();
        printf("packet[%d]: data array, lc=%d, int32[%d][%d], channel: %d\n",sn,lc,data.get_row_size(),data.get_col_size(),channel);
        box_lc = lc;
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

        case 'h':// manually toggle whether the device should behave as if the host app running
            if(host_on){
                box->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_DC);
                printf("host off\n");
            }else{
                box->post_message(VS_MSG_ROUTE_APP,VS_MSG_HOST_ON);
                printf("host on\n");
            }
            host_on = !host_on;
            break;

        }
    }
}


int main(int argc,char*argv[]){
    char ip_str[20] = "127.0.0.1";
    int r;

    // use the IP address in argument if possible
    if(argc>=2){
        snprintf(ip_str,20,"%s",argv[1]);
    }
    
    printf("GUI Interaction Example\n");

    // a thread to handle console keyboard input
    std::thread input_thread(input_loop);

    // 'scamp5d_packet_switch' the object to decode the incoming packets and pass them
    // to the corresponding handler function registered.
    packet_switch = new vs_packet_decoder;

    setup_packet_switch();

    /// Initialize the device interface
    box = new scamp5d_usb();
    r = box->open("",-1);
    //box = new scamp5d_tcp();
    //r = box->open("127.0.0.1",27888);
    if(r){
        fprintf(stderr,"<Error> failed to open device!\n");
        exit(-1);
    }else{
        printf("<Device Ready>\n------------------------------------------------\n");
    }

    /// Start the TCP Proxy
    proxy = new scamp5d_proxy;
    proxy->open(ip_str,27725);

    /// Setup how the interface handles packet received from the device
    // Here the "free_packet" callback is used, which needs to 'free' the packet after use.
    box->on_free_packet([&](uint8_t*packet,size_t packet_size){

        // process the packet
        packet_to_discard = false;
        packet_switch->decode_packet(packet,packet_size,box->get_packet_counter());
        if(packet_to_discard){
            free(packet);
            return;
        }

        // forward to proxy, and deal with proxy bottle neck
        const size_t warning_size = 512*1024;
        const size_t discard_size = 10*1024*1024;
        size_t queue_size = proxy->get_bytes_in_queue();

        if(queue_size>discard_size){
            printf("[scamp5d_proxy] warning: queue over %d bytes, packet discarded!!!\n",queue_size);
            // don't broadcast, just free
            free(packet);
        }else{
            if(queue_size>warning_size){
                printf("[scamp5d_proxy] warning: over %d bytes of data queued!\n",queue_size);
            }
            // broadcast through the proxy and free the packet
            proxy->broadcast(packet,packet_size,true);
        }

    });

    /// Setup how the proxy handles packet received from its clients
    proxy->on_receive_packet([&](const uint8_t*packet,size_t bytes){
        //printf("client -> device: %d, %s\n",bytes,HEX_STR(packet,(bytes>16)? 16:bytes));
        printf("client -> device: %d bytes\n",bytes);
        box->write(packet,bytes);
    });


    /// Setup GUI Graphics API
    vs_interaction_proxy_setup(packet_switch,proxy);

    /// Main Loop
    while(quit==false){
        box->routine();
        proxy->routine();
        sleep_msec(1);
    }

    vs_interaction_proxy_clear();

    proxy->close();
    delete proxy;

    box->close();
    delete box;

    delete packet_switch;

    input_thread.join();

    return 0;
}
