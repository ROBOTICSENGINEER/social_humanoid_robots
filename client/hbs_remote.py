#!/usr/bin/env python3
# Code Author: Stephen Brooks

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
import requests
import logging

from time import sleep


WELCOME = """
Welcome to the Robo-Sense control terminal, sponsored by Dewritos:
    "If you want to fuel your pro gaming workouts the right way, choose Dewritos!"
Please enter a command:
    1)  Detect distance of nearest object (in centimeters) in front of the robot
    2)  Speak
    3)  Shake head
    4)  Read the ambient temperature (in fahrenheit)
    5)  Exit the program
"""

API_URI = 'http://3.16.160.44:5000'
VALID_COMMANDS = {
    1: 'readsonar',
    2: 'speak',
    3: 'shakehead',
    4: 'readtemp'
}


def wait_for_data():
    logger.info("Sending request for data from the sensor.")
    req = requests.get(API_URI + '/api/getdata')
    if req.status_code == 200 and len(req.content) > 0:
        print("Sensor data recieved.")
        return req.json()
    else:
        logger.warning("No data recieved from sensor.")
        return None


def send_command(user_input, tts=None):
    if tts is not None:
        logger.info('Sending GET request with data %s ', tts)
        req = requests.get(''.join([API_URI, '/api/speak?tts=', tts]))
    else:
        logger.info('Sending GET request for \"%s\" command.', user_input)
        req = requests.get(''.join([API_URI, '/api/', VALID_COMMANDS.get(user_input)]))
    
    return req.status_code


def check_input(user_input):
    if user_input == 5:
        logger.warning('User wishes to exit. Shutting down...')
        print('Goodbye.')
        sys.exit(0)
    elif not user_input in VALID_COMMANDS:
        logger.warning('User input %s is not a valid command.', user_input)
        print('Your input is not a valid command. Try again: ')
        return None
    else:
        logger.info('User input %s was accepted.', user_input)
                           
        if user_input == 2:
            sys.stdout.write('Please provide text for the robot to speak: ')
            tts = input()
            return send_command(user_input, tts)
        else:
            status = send_command(user_input)
            if status == 200 and user_input in [1,4]:
                print('Waiting a maximum of 10 seconds for data from the sensor...\n')
                for i in range(10):
                    data = wait_for_data()
                    if data is not None:
                        print('Data recieved: {}'.format(data))
                        break
                    else:
                        logger.warning("Trying to get data again.")
                        sleep(1)
                if data is None:
                    print('No data recieved from the sensor.')
            return status
    logger.error('Something went wrong...')
    return None


def main():
    logger.info('Application has started.')
    
    print(WELCOME)
    while True:
        try:
            sys.stdout.write('Command: ')
            user_input = input()
            resp = check_input(int(user_input))
            if resp == None:
                pass
            elif resp != 200:
                logger.error('The request was unsuccessful. Status code: %s ', resp)
                print('The API call failed. Try again... ')
        except ValueError:
            logger.warning('User input %s is not a valid command.', user_input)
            sys.stdout.write('Your input is not a valid command. Try again: ')
            pass
    return 0


if __name__ == '__main__':
    # start_logging()
    logging.basicConfig(filename='robot_remote.log',
                        level=logging.DEBUG,
                        format='%(asctime)s : %(levelname)s\t %(message)s')
    logger = logging.getLogger(__name__)
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt detected. Shutting down...')
        sys.exit(0)
