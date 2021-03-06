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
#include <sys/types.h>
#include <queue>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"

// Custom
#include "Request.hpp"


static int fd;
static char const* iic_dev_0 = "/dev/i2c-0";
static char const* iic_dev_1 = "/dev/i2c-1";
std::queue<Request> work_queue, completed_queue;


bool write_req(Request* job) {
    ROS_DEBUG("A write request has been made for device %X.", job->get_id());

    if (job->get_bus() == 0) {
        if ((fd = open(iic_dev_0, O_RDWR)) < 0) {
            ROS_ERROR("Cannot open I2C bus 0."); 
            return false;
        }
    } else {
        if ((fd = open(iic_dev_1, O_RDWR)) < 0) { 
            ROS_ERROR("Cannot open I2C bus 1."); 
            return false;
        }
    }

    uint8_t to_write[job->data.size()] = {0};
    uint8_t dev_addr = job->get_id();
    for (int i = 0; i < job->data.size(); i++) {
        to_write[i] = job->data.at(i);
    }

    ioctl(fd, I2C_SLAVE, dev_addr); 
    /* send the single write command and data. */
    if (write(fd, to_write, job->data.size()) != job->data.size()) {
        job->num_attempts++;
        ROS_ERROR("Failed to write to I2C bus. The bus manager will try twice more.");
        for (int i = 0; i < 2; i++) {
            job->num_attempts++;
            /* wait just to be safe. */
            usleep(70000);
            if (write(fd, to_write, job->data.size()) != job->data.size()) {
                ROS_ERROR("Failed to write to the I2C bus. Trying again.");
            }
            else {
                close(fd);
                return true;
            }
        }
        ROS_WARN("Failed to write to the I2C bus three times. Returning unsuccessful status.");
        close(fd);
        return false;
    } 
    else {
        job->set_status(true);
        close(fd);
        return true;
    }
}


bool read_req(Request* job) {
    ROS_DEBUG("A read request has been made for device %X.", job->get_id());
    
    if (job->get_bus() == 0) {
        if ((fd = open(iic_dev_0, O_RDWR)) < 0) {
            ROS_ERROR("Cannot open I2C bus 0."); 
            return false;
        }
    } else {
        if ((fd = open(iic_dev_1, O_RDWR)) < 0) { 
            ROS_ERROR("Cannot open I2C bus 1."); 
            return false;
        }
    }    

    uint8_t is_read[job->data.size()] = {0};
    uint8_t read_reg[job->data.size()] = {0};
    uint8_t dev_addr = job->get_id();

    for (int i = 0; i < job->data.size(); i++) {
        is_read[i] = job->data.at(i);
    }

    ioctl(fd, I2C_SLAVE, dev_addr);
    /* write to the device in preparation for read. */
    write(fd, is_read, 1);
    /* wait for next communication. */
    usleep(70000);

    /* read data from requested registers. */
    if (read(fd, read_reg, job->data.size()) != job->data.size()) {
        job->num_attempts++;
        ROS_WARN("Failed to read from I2C bus. The bus manager will try twice more.");
        for (int i = 0; i < 2; i++) {
            job->num_attempts++;
            usleep(70000);
            if (read(fd, read_reg, job->data.size()) != job->data.size())
                ROS_WARN("Failed to read from the I2C bus. Trying again.");
            else {
                close(fd);
                return true;
            }
        }
        ROS_ERROR("Failed to read from the I2C bus three times. Returning unsuccessful status.");
        close(fd);
        return false; 
    }
    else {
        /* copy the device data into job.data. */
        job->data.resize(job->data.size());
        memcpy(&job->data[0], &read_reg[0], job->data.size() * sizeof(uint8_t));
        job->set_status(true);
        close(fd);
        return true;
    }
}


bool handle_req(hbs2::i2c_bus::Request &req, hbs2::i2c_bus::Response &res) {
    Request request = Request(req.request/*, req.size*/, req.bus);
    
    /* check to see if the request is for status and serve it first if so. */
    if (request.get_type() == "status") {
        ROS_DEBUG("A request has been made for the status of device %X.", req.request[1]);
        /* find the job that the request for status is for. it should be in the completed queue. */
        for (int i = 0; i < completed_queue.size(); i++) {
            Request job = completed_queue.front();
            if (job.get_id() == req.request[1]) {
                /* remove the job from the completed queue since it will no longer be asked for status and break. */
                completed_queue.pop();
                res.data = job.data;
                return true;
            }
            else {
                /* this job is not the one we are looking for. back of the line. */
                completed_queue.pop();
                completed_queue.push(job);
            }    
        }
        return false;
    }
    /* otherwise, put the job on the work queue. */
    else { work_queue.push(request); }
    
    /* always empty the work queue before returning from the callback. */
    while (!work_queue.empty()) {

        // TODO sort on priority

        Request job = work_queue.front();
        work_queue.pop();
        if (job.get_type() == "read") { 
            if (read_req(&job)) {
                /* the read request was successful. */
                completed_queue.push(job); 
                res.data = job.data;
            }
            else {
                /* the read request was NOT sucessful. */
                if (job.num_attempts < 6) {
                    ROS_WARN("Putting request for device %X back onto work queue.", job.get_id());
                    work_queue.push(job);
                }
                else {
                    ROS_ERROR("Request for device %X is being dropped from the queue.", job.get_id());
                }
                ROS_ERROR("Read request for device %X failed.", job.get_id());
            }
        }
        else if (job.get_type() == "write") { 
            if (write_req(&job)) {
                /* the write request was successful. */
                completed_queue.push(job); 
            }
            else {
                /* push the job back onto the queue to try again. */
                work_queue.push(job);
                ROS_ERROR("Write request for device %X failed.", job.get_id());
            }

        }
    }
    return true;
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
    ros::init(argc, argv, "i2c_bus_mgr_srv");
    ros::NodeHandle n;

    ros::ServiceServer srv = n.advertiseService("i2c_srv", handle_req);
    ROS_INFO("ROS I2C Bus Manager has started.");
    ros::spin();
    return 0;
}
