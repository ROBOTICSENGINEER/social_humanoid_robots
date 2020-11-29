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
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>

// Installed
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Infos.hpp>

// ROS
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "hbs2/iot.h"


#define MAX_FILE_LENGTH 20000


char *m_pBuffer = NULL;
size_t m_Size = 0;

/*  Class: Response
    desc: A class to store contents of an HTTP respone
    attrs:
        code: HTTP response code
        size: Size of the response body in bytes
        content: HTTP response body content
*/
class Response {
    public:
        long code;
        int size;
        std::string content;
};


void* Realloc(void* ptr, size_t size)
{
  if(ptr)
    return realloc(ptr, size);
  else
    return malloc(size);
};


size_t WriteMemoryCallback(char* ptr, size_t size, size_t nmemb)
{
  // Calculate the real size of the incoming buffer
  size_t realsize = size * nmemb;
  
  // (Re)Allocate memory for the buffer
  m_pBuffer = (char*) Realloc(m_pBuffer, m_Size + realsize);
  
  // Test if Buffer is initialized correctly & copy memory
  if (m_pBuffer == NULL) {
    realsize = 0;
  }
  
  memcpy(&(m_pBuffer[m_Size]), ptr, realsize);
  m_Size = realsize;
  
  // return the real size of the buffer...
  return realsize;
};

/*  Function: rest_req (REST Request)
    desc:   A funciton that performs an HTTP request with libcurl + curlpp
    inputs:
        &resp: HTTP response object
        uri: Uniform Resource Locator for the API call
    outputs:
        &resp: HTTP response object
*/
void rest_req(Response &resp, std::string uri) {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;

    request.curlpp::Easy::setOpt(curlpp::options::Url(uri));
    request.curlpp::Easy::setOpt(curlpp::options::Port(5000));
    curlpp::types::WriteFunctionFunctor functor(WriteMemoryCallback);
    curlpp::options::WriteFunction *writefunc = new curlpp::options::WriteFunction(functor);

    request.setOpt(writefunc);
    request.perform();

    curlpp::infos::ResponseCode::get(request, resp.code);
    resp.size = m_Size;
    if (m_Size == 1) { resp.content = m_pBuffer[0]; }
    else { resp.content = ++m_pBuffer; m_pBuffer--; }
}

/*  Function: begin
    desc:   A function that is called from main in a loop at 2Hz rate that will perform and REST request 
            and controls the flow of the Node.
    inputs:
        &n: ROS NodeHandle for this Node
    outputs:
        iot_client.call: sends a request to the iot service for actions and/or data collection
        iot_srv.response.data: response to client with data from sensor        
*/
void begin(ros::NodeHandle &n) {
    m_pBuffer = (char*) malloc(MAX_FILE_LENGTH * sizeof(char));
    Response resp;
    std::string get_cmd_uri = "http://3.16.160.44/api/getcommand";
    std::string get_tts_uri = "http://3.16.160.44/api/gettts";
    std::string set_resp_uri = "http://3.16.160.44/api/setdata?data=";
    m_pBuffer = NULL;
    m_Size = 0;
    resp.content = "";
    resp.size = 0;
    resp.code = 0;

    ros::ServiceClient iot_client = n.serviceClient<hbs2::iot>("iot_srv");
    hbs2::iot srv_iot;

    rest_req(resp, get_cmd_uri);

    switch (m_pBuffer[0]) {
        case 1: {
            ROS_DEBUG("A request to read the sonar sensor has been made.");
            /*  send request to iot_srv
                req.request.command = 1

                check for success and return data back to api client
                    create rest_req(resp, set_resp_uri)
            */
            srv_iot.request.command = 1;
            if (iot_client.call(srv_iot)) {
                ROS_INFO("Sending sonar data to remote IoT client.");
                rest_req(resp, set_resp_uri + std::to_string(srv_iot.response.data));
            }
            else { ROS_ERROR("Request to iot_srv for reading sonar failed."); }
            break;
        }
        case 2: {
            ROS_DEBUG("A request for TTS has been made.");
            rest_req(resp, get_tts_uri);
            /*  send request to iot_srv
                req.request.command = 2
                req.request.text = resp.content

                check for success
                else error
            */
            srv_iot.request.command = 2;
            srv_iot.request.text = resp.content;
            if (iot_client.call(srv_iot)) { ROS_INFO("Request for TTS completed."); }
            else { ROS_ERROR("Request to iot_srv for TTS has failed."); }
            break;
        }
        case 3: {
            ROS_DEBUG("A request to shake the robot's head has been made.");
            /*  send request to iot_srv
                req.request.command = 3
            
                check for success
                else error
            */
            srv_iot.request.command = 3;
            if (iot_client.call(srv_iot)) { ROS_INFO("Request to shake head completed."); }
            else { ROS_ERROR("Request to iot_srv to shake head has failed."); }
            break;
        }
        case 4: {
            ROS_DEBUG("A request to read the temperature sensor has been made.");
            /*  send request to iot_srv
                req.request.command = 4

                check for success and return data back to api client
                    create rest_req(resp, set_resp_uri)
            */
            srv_iot.request.command = 4;
            if (iot_client.call(srv_iot)) {
                ROS_INFO("Sending temperature data to remote IoT client.");
                rest_req(resp, set_resp_uri + std::to_string(srv_iot.response.data));
            }
            else { ROS_ERROR("Request to iot_srv for reading sonar failed."); }
            break;
        }
        default:
            ROS_DEBUG("[ROS IoT] No action taken.");
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
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "iort");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    
    while(ros::ok()) {
        begin(n);
        ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}
