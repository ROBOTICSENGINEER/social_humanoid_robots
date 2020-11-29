#!/usr/bin/env python3
# Author(s): Stephen Brooks
# Modified by Mohsen Jafarzadeh at December 19, 2018


'''

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


'''



import os
import sys
import logging
from flask import Flask, request

COMMAND_FILE_PATH = r'/opt/.git/capstone-hhri/iort/hbs/api/command.bin'
SPEECH_KW_PATH = r'/opt/.git/capstone-hhri/iort/hbs/api/tts.txt'
RETURN_DATA_PATH = r'/opt/.git/capstone-hhri/iort/hbs/api/data.txt'
app = Flask(__name__)

""" Error return 0
    Success return 255
    Requests:
        1   :   readsonar
        2   :   speak
        3   :   shakehead
        4   :   readtemp
"""

@app.route('/api')
def index():
    return "Hello World!"


@app.route('/api/getcommand')
def get_command():
    if os.path.exists(COMMAND_FILE_PATH):
        with open(COMMAND_FILE_PATH, 'rb') as cmd_file:
            logger.info('Reading from command file.')
            cmd = cmd_file.read(1)
        # truncate file after returning its data to robot
        with open(COMMAND_FILE_PATH, 'wb') as cmd_file:
            cmd_file.write(b'\x00')
        return cmd
    else:
        logger.error('Command file does not exist.')
        return b'\x00'


@app.route('/api/gettts')
def get_tts():
    if os.path.exists(SPEECH_KW_PATH):
        with open(SPEECH_KW_PATH, 'r') as spch_file:
            logger.info('Reading from tts file.')
            return spch_file.readline()
        # truncate file after returning its data to robot
        open(SPEECH_KW_PATH, 'w').close()
    else:
        logger.error('TTS file does not exist.')
        return b'\x00'


@app.route('/api/getdata')
def get_data():
    logger.info("Recieved a request for sensor data.")
    if os.path.exists(RETURN_DATA_PATH):
        with open(RETURN_DATA_PATH, 'r+') as data_file:
            data = data_file.readline()
            logger.info("Sending \"%s\" to client.", data)
        # truncate file in case of subsequent requests
        open(RETURN_DATA_PATH, 'w').close()
        return data
    else:
        logger.error("Data file does not exist.")
        return b'\x00'


@app.route('/api/setdata')
def set_data():
    logger.info("Data from robot incomming. Writing to data file.")
    with open(RETURN_DATA_PATH, 'w') as data_file:
        logger.info("Writing \"%s\" to data file.", request.args['data'])
        sensor_data = int(request.args['data']) % 2**16
        data_file.write(str(sensor_data))
    return b'\xFF'


@app.route('/api/readsonar')
def read_sonar():
    print("function: sonar")
    logger.info("Request to read sonar sensor has been made.")
    with open(COMMAND_FILE_PATH, 'wb') as cmd_file:
        logger.info('Writing 0x01 to command file.')
        cmd_file.write(b'\x01')
    print("return: sonar")
    return b'\xFF'


@app.route('/api/readtemp')
def read_temp():
    print("function: temperature")
    logger.info("Request to read temperature sensor has been made.")
    with open(COMMAND_FILE_PATH, 'wb') as cmd_file:
        logger.info('Writing 0x04 to command file.')
        cmd_file.write(b'\x04')
    print("return: temperature")
    return b'\xFF'


@app.route('/api/speak')
def speak():
    print("function: speak")
    logger.info('Request to use TTS has been made')
    with open(COMMAND_FILE_PATH, 'wb') as cmd_file:
        logger.info('Writing 0x02 to command file.')
        cmd_file.write(b'\x02')
    with open(SPEECH_KW_PATH, 'w') as spch_file:
        logger.info('Writing \"%s\" to TTS file.', request.args['tts'])
        spch_file.write(request.args['tts'])
    print("return: speak")
    return b'\xFF'


@app.route('/api/shakehead')
def shake_head():
    print("function: shake head")
    logger.info('Request to shake head has been made.')
    with open(COMMAND_FILE_PATH, 'wb') as cmd_file:
        logger.info('Writing 0x03 to command file.')
        cmd_file.write(b'\x03')
    print("returen: shake head")
    return b'\xFF'


if __name__ == '__main__':
    print("start main")
    logging.basicConfig(filename='robot_api.log',
                        level=logging.DEBUG,
                        format='%(asctime)s : %(levelname)s\t %(message)s')
    logger = logging.getLogger(__name__)
    logger.info('API starting.')
    print("logger.info is ok!")
    app.run(debug=True, host='0.0.0.0')
    print("main success")
