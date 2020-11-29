/*      Touch behavior node, subscriber to MPR121 hardware node publisher       


Cite as:

IEEE
M. Jafarzadeh, S. Brooks, S. Yuc, B. Prabhakaran, and Y. Tadesse, " A wearable sensor vest for social 
humanoid robots with GPGPU, IoT, and modular software architecture," Robotics and Autonomous Systems.


Bibtex
@article{jafarzadeh2020wearable, 
title={A wearable sensor vest for social humanoid robots with GPGPU, IoT, and modular software architecture}, 
author={Jafarzadeh, Mohsen and Brooks, Stephen and Yu, Shimeng and Prabhakaran, Balakrishnan and Tadesse, Yonas}, 
journal={Robotics and Autonomous Systems}, 
publisher={Elsevier} }


Copyright (c) 2020 Mohsen Jafarzadeh and Stephen Brooks. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions 
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must display the following 
acknowledgement: This product includes software developed by Mohsen Jafarzadeh, Stephen Brooks, 
Sharon Choi, Manpreet Dhot, Mark Cordova, Luis Hall-Valdez, and Shimeng Yu.

4. Neither the name of the Mohsen Jafarzadeh nor the names of its contributors may be used to endorse or 
promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MOHSEN JAFARZADEH "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL MOHSEN JAFARZADEH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// System
#include <stdio.h>
#include <unistd.h>
#include <vector>


// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "hbs2/led.h"
#include "std_msgs/UInt8.h"


// declare global NodeHandle
ros::NodeHandlePtr n = NULL;

/*  Function: touch_callback
    desc: If a touch has occurred at the correct pin, the humanoid will say thank you
    inputs:
        &msg: contains the current data on the tpc_touch queue
    outputs:
        tts_client.call: Sends a request to the TTS service
*/
void touch_callback(const std_msgs::UInt8::ConstPtr& msg) {
    if (msg->data == 1 || msg->data == 3) { 
        ROS_DEBUG("Touch occurred at pin %u", msg->data);

        ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
        ros::ServiceClient led_client = n->serviceClient<hbs2::led>("led_srv");
        hbs2::tts srv_tts;
        hbs2::led srv_led;

        srv_tts.request.text = "Thank you";
        srv_led.request.color = 4;
        if (tts_client.call(srv_tts) && led_client.call(srv_led)) { ROS_INFO("TTS service call from touch behavior completed successfully."); }
        else { ROS_ERROR("TTS service call from touch behavior failed."); }

        sleep(1);   // wait to turn LEDs off

        srv_led.request.color = 0;
        if (led_client.call(srv_led)) { ROS_INFO("LED service call from touch behavior completed successfully."); }
        else { ROS_ERROR("LED service call from touch behavior failed."); }
    }

}

/*  Function: main
    desc: Entry point for the Node
    inputs:
        argc: count of command line arguments
        argv: array of command line arguments
    outputs:
        int: always 0 if exits gracefully
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "touch");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_touch", 5, touch_callback);

    ros::spin();

    return 0;
}
