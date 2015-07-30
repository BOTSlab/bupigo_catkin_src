#!/usr/bin/env python

"""
    AV - Modified to incorporate ARbot, RTbot, and BuPiGo control functions.

    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
#from bupigo_msgs.msg import Blobs, Blob
from serial import Serial

SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            self.port.rtscts = True
            self.port.dsrdtr = True
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')
        #print value
        return value
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        got_lock = self.mutex.acquire()
        if not got_lock:
            return None
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()
        
        try:
            return int(value)
        except:
            print "Exception executing command: " + cmd
            if cmd is 'g':
                return 0
            elif cmd is 'b':
                return 57600
            else:
                return None
        

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        got_lock = self.mutex.acquire()
        if not got_lock:
            return []
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        got_lock = self.mutex.acquire()
        if not got_lock:
            return 0
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'   
    
    # AV - Start
    def bupigo_read_odometry(self):
        ''' Return (x, y, theta) tuple with values in metres and radians.
        '''
        values = self.execute_array('o')
        if len(values) != 3:
            print "Expecting three values back from 'o' command"
            raise SerialException
            return None
        else:
            x = values[0] / 1000.0
            y = values[1] / 1000.0
            theta = values[2] / 1000.0
            return (x, y, theta)

#    def bupigo_set_speeds(self, speeds):
#        return self.execute_ack('s %d' %(speeds.left_speed, speeds.right_speed))

    def bupigo_set_velocity(self, forwardSpeed, angularSpeed):
        ''' Speeds are given in m/sec and rads/sec
        '''
        intForward = int(forwardSpeed * 1000.0)
        intAngular = int(angularSpeed * 1000.0)
        return self.execute_ack('v %d %d' %(intForward, intAngular))

    def bupigo_set_servo(self, angle):
        return self.execute_ack('a %d' %(angle))

    def rtbot_read_odometry(self):
        ''' Return (x, y, theta) tuple with values in metres and radians.
        '''
        values = self.execute_array('o')
        if len(values) != 3:
            print "Expecting three values back from 'o' command"
            raise SerialException
            return None
        else:
            x = values[0] / 1000.0
            y = values[1] / 1000.0
            theta = values[2] / 1000.0
            return (x, y, theta)

    def rtbot_set_motors(self, stp_for_rev_lft_rgt):
        ''' Set the motors to stop (0), go forward (1), reverse (2), go left (3), or go right (4)
        '''
        return self.execute_ack('m %d' %(stp_for_rev_lft_rgt))

    def arbot_read_odometry(self):
        ''' Return (x, y, theta) tuple with values in metres and radians.
        '''
        values = self.execute_array('o')
        if len(values) != 3:
            print "Expecting three values back from 'o' command"
            raise SerialException
            return None
        else:
            x = values[0] / 1000.0
            y = values[1] / 1000.0
            theta = values[2] / 1000.0
            return (x, y, theta)

    def arbot_set_play_led(self, on):
        '''Set the LED next to the play button on the ARbot'''
        if on:
            return self.execute_ack('f 1')
        else:
            return self.execute_ack('f 0')

    def arbot_set_advance_led(self, on):
        '''Set the LED next to the advance button on the ARbot'''
        if on:
            return self.execute_ack('g 1')
        else:
            return self.execute_ack('g 0')

    def arbot_set_velocity(self, forwardSpeed, angularSpeed):
        ''' Speeds are given in m/sec and rads/sec
        '''
        intForward = int(forwardSpeed * 1000.0)
        intAngular = int(angularSpeed * 1000.0)
        return self.execute_ack('v %d %d' %(intForward, intAngular))

    def get_blobs(self):
        """ Get the color data from pixy
        """
        number_of_blobs = int(self.execute('g'))
        blobs = Blobs()
        blobs.header.frame_id = 'base_link'
        blobs.blob_count = number_of_blobs
        if number_of_blobs is 0:
            return blobs
        for i in range(0, number_of_blobs):
            blob_data = self.execute_array('h {0}'.format(i))
            blob = Blob()
            #print blob_data[0]
            #print blob_data[1]
            #print blob_data[2]
            #print blob_data[3]
            #print blob_data[4]
            blob.type = blob_data[0]
            blob.x = int(blob_data[1])
            blob.y = int(blob_data[2])
            blob.area = int(blob_data[3]) * int(blob_data[4])
            blob.left = blob.x - int(blob_data[3])/2
            blob.right = blob.x + int(blob_data[3])/2
            blob.bottom = blob.y + int(blob_data[4])/2
            blob.top = blob.y - int(blob_data[4])/2
            blobs.blobs.append(blob)
        return blobs

    # AV - Stop
    
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)                          

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'))

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')
    
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return self.execute_ack('m %d %d' %(right, left))
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
            
    def analog_read(self, pin):
        return self.execute('a %d' %pin)
    
    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))
    
    def digital_read(self, pin):
        return self.execute('d %d' %pin)
    
    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))
    
    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''        
        return self.execute_ack('s %d %d' %(id, min(SERVO_MAX, max(SERVO_MIN, degrees(pos)))))
    
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''        
        return radians(self.execute('t %d' %id))

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin);
    
#    def get_maxez1(self, triggerPin, outputPin):
#        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
#            sensor connected to the General Purpose I/O lines, triggerPin, and
#            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
#            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
#            power up, otherwise it wont range correctly for object less than 6
#            inches away! The sensor reading defaults to use English units
#            (inches). The sensor distance resolution is integer based. Also, the
#            maxsonar trigger pin is RX, and the echo pin is PW.
#        '''
#        return self.execute('z %d %d' %(triggerPin, outputPin)) 
 

""" Basic test for connectivity """
if __name__ == "__main__":
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.
        
    baudRate = 57600

    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()
     
    print "Sleeping for 1 second..."
    time.sleep(1)   
    
    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    
    print "Connection test successful.",
    
    myArduino.stop()
    myArduino.close()
    
    print "Shutting down Arduino."
    
