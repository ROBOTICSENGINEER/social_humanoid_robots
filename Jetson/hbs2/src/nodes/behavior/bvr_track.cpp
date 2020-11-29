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
#include <unistd.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/servo.h"
#include "std_msgs/UInt16MultiArray.h"


const int _SERVO_SPEED_ = 20;
const int _SERVO_POS_CHANGE_ = 20;

ros::NodeHandlePtr n = NULL;
int last_position = 90; // starting position at initialization is 90 degrees

/*  Function: track_callback
    desc:   Callback function for the IR sensor topic. Sends requests to the maestro service to move servos.
    inputs:
        &msg:   Array containing the current message on the IR sensor topic.
    outputs:
        servo_client.call: sends a request to the maestro service
*/
void track_callback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    ros::ServiceClient servo_client = n->serviceClient<hbs2::servo>("servo_srv");
    hbs2::servo srv_servo;

    int index = 0;

    // search through the topic queue and find the minimum
    for (int i = 1; i < msg->data.size(); i++) {
        if (msg->data.at(i) < msg->data.at(index))
            index = i;
    }

    switch(index) {
        case 0: {
            ROS_INFO("Object is centered.");
            int diff = 0;
            if (last_position == 90) { break; }
            else { 
                diff = 90 - last_position;
                if (diff > 0) {
                    srv_servo.request.command = 1;
                    srv_servo.request.position = last_position + _SERVO_POS_CHANGE_;
                    last_position += _SERVO_POS_CHANGE_;
                }
                else {
                    srv_servo.request.command = 1;
                    srv_servo.request.position = last_position - _SERVO_POS_CHANGE_;
                    last_position -= _SERVO_POS_CHANGE_;
                }
                servo_client.call(srv_servo);
            }
            break;
        }
        case 1: {
            ROS_INFO("Object is right of center.");
            if (last_position < 140) {
                // Make sure that the servo speed is set properly before sending the position request
                srv_servo.request.command = 2;
                srv_servo.request.speed = _SERVO_SPEED_;
                servo_client.call(srv_servo);
                srv_servo.request.command = 1;
                srv_servo.request.position = last_position + _SERVO_POS_CHANGE_;
                last_position += _SERVO_POS_CHANGE_;
                servo_client.call(srv_servo);
            }
            break;
        }
        case 2: {
            ROS_INFO("Object is left of center.");
            if (last_position > 40) {
                // Make sure that the servo speed is set properly before sending the position request
                srv_servo.request.command = 2;
                srv_servo.request.speed = _SERVO_SPEED_;
                servo_client.call(srv_servo);            
                srv_servo.request.command = 1;
                srv_servo.request.position = last_position - _SERVO_POS_CHANGE_;
                last_position -= _SERVO_POS_CHANGE_;
                servo_client.call(srv_servo); 
            }
            break;
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
    ros::init(argc, argv, "track");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_track", 5, track_callback);

    ros::spin();

    return 0;
}
