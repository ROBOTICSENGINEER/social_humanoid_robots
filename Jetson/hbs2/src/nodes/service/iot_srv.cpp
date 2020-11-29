/*
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
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <functional>

// ROS
#include "ros/ros.h"
#include "hbs2/iot.h"
#include "hbs2/sonar.h"
#include "hbs2/tts.h"
#include "hbs2/servo.h"
#include "hbs2/temp.h"

// Declare a global NodeHandle
ros::NodeHandlePtr n = NULL;

/*  Function: handle_req (handle request)
    desc: Callback function for the IoT service. A proxy service for the IoRT behavior node.
    inputs:
        &req: req.command: identifies which service to call on behalf of the IoRT node
            command:
                |   1: call sonar service   |
                |   2: call tts service     |
                |   3: call servo service   |
                |   4: call to temp service |
    outputs:
        sonar_client.call: sends a request to the sonar service to obtain a measurement
        tts_client.call: sends a request to the tts service
        servo_client.call: sends a request to the servo service to shake the robot's head
        temp_srv.call: sends a request to the temperature service to obtain a measurement
*/
bool handle_req(hbs2::iot::Request &req, hbs2::iot::Response &res) {
    ROS_INFO("Serving request from [bvr_iort] node.");
    
    ros::ServiceClient sonar_client = n->serviceClient<hbs2::sonar>("sonar_srv");
    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    ros::ServiceClient servo_client = n->serviceClient<hbs2::servo>("servo_srv");
    ros::ServiceClient temp_client = n->serviceClient<hbs2::temp>("temp_srv");

    hbs2::sonar srv_sonar;
    hbs2::tts srv_tts;
    hbs2::servo srv_servo;
    hbs2::temp srv_temp;

    switch (req.command) {
        case 1: {
            ROS_INFO("Serving request to read sonar sensor.");
            if (sonar_client.call(srv_sonar)) {
                res.data = srv_sonar.response.data;
                ROS_INFO("Sonar measurement service request call from the IoT service has completed successfully.");
                return true;
            }
            else {
                ROS_ERROR("Request to read sonar sensor failed in iot_srv.");
                return false;
            }
        }
        case 2: {
            ROS_INFO("Serving request for TTS.");
            srv_tts.request.text = req.text;
            if (tts_client.call(srv_tts)) {
                ROS_INFO("TTS service call from the IoT service has completed successfully.");
                return true;
            }
            else {
                ROS_ERROR("TTS Request failed in iot_srv.");
                return false;
            }
        }
        case 3: {
            ROS_INFO("Serving request to shake head.");
            srv_servo.request.command = 2;
            srv_servo.request.speed = 100;
            if (servo_client.call(srv_servo)) { ROS_DEBUG("Servo speed has changed to 100 RPM."); }
            else {
                ROS_ERROR("Request to change speed of the servo has failed in iot_srv.");
                return false;
            }

            // For even iterations, turn head left, else turn right
            for(int i = 1; i < 5; i++) {
                srv_servo.request.command = 1;
                if (i % 2)
                    srv_servo.request.position = 60;
                else
                    srv_servo.request.position = 120;
                servo_client.call(srv_servo);
                // wait some time for the previous movement to complete
                usleep(300000);
            }

            srv_servo.request.position = 90;
            if(servo_client.call(srv_servo)) {
                ROS_INFO("Request to shake head has succeeded.");
                return true;
            }
            else {
                ROS_ERROR("Request to shake head has failed in iot_srv.");
                return false;
            }
        }
        case 4: {
            ROS_INFO("Serving request to read temperature sensor.");
            if (temp_client.call(srv_temp)) {
                res.data = srv_temp.response.data;
                ROS_INFO("Temperature measurement service request call from the IoT service has completed successfully.");
                return true;
            }
            else {
                ROS_ERROR("Request to read temperature sensor failed in iot_srv.");
                return false;
            }
        }
        default:
            break;
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
    ros::init(argc, argv, "iot_srv");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::ServiceServer srv = n->advertiseService("iot_srv", handle_req);

    ROS_INFO("ROS IoT Service has started.");
    ros::spin();
    return 0;
}
