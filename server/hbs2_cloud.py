#!/usr/bin/env python3
# Author(s): Stephen Brooks
# Modified by Mohsen Jafarzadeh at December 19, 2018

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