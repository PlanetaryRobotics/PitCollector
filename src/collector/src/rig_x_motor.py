from tools.labjack_tools import *

import rospy
import time
import numpy as np
from sensor_msgs.msg import Imu


def x_axis_find_switch(self):
    # if the x axis does not have any switches activated
    # first set the x direction switch to forward
    # second activate motor command
    # third read pins until you hit a switch on the x-axis
    # fourth shut off motor command
    # the x axes pins are F0, F1, F2

    # set E1 pin to High
    # moving forward, at them moment, is safer than backwards. 
    ans = str(raw_input('No switch active on X axis...move forward until you hit a switch? (y/n):'))
    if ans in ['yes', 'y']:
        writeDIO('E1',1)
        if self.check_for_positive_pins(['F0','F1','F2']):
            print('x_axis_find_switch: switch triggered stopping motors')
            return True
        else:
            while self.check_for_positive_pins(['F0','F1','F2']) == False:
                #set E0 to high (x-motor go)
                writeMotorsDIO('E0',1)
                pass
            print('x_axis_find_switch: X switch activated')
            #set E0 to low
            writeMotorsDIO('E0',0)
            return True
    elif ans in ['no', 'n']:
        ans2 = str(raw_input('move backwards until you hit a switch? (y/n):'))
        if ans2 in ['yes','y']:
            #set motor to backwards until you hit a positive switch
            writeDIO('E1',0)
            if self.check_for_positive_pins(['F0','F1','F2']):
                print('x_axis_find_switch: switch triggered, stopping motors')
                return True
            else:
                while self.check_for_positive_pins(['F0','F1','F2']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                    pass
                print('x_axis_find_switch: X switch activated')
                #set E0 to low
                writeMotorsDIO('E0',0)
                return True
        else:
            print("x_axis_find_switch: aborting")
            sys.exit(0)
            return False
    else:
        print("x_axis_find_switch: aborting")
        sys.exit(0)
        return False

def x_axis_go_to(self, pos):
    #where am I at?
    #which pin am I going to?
    #what direction is this pin?

    #if you're at home
    #print('in x_axis_go_to')

    if pos in  ['F0','F1','F2']:
        pass
    else:
        print('x_axis_go_to: invalid go to pin')
        sys.exit(0)
        return False

    if self.check_for_positive_pins(['F0']):
        if pos == 'F0':
            return True
        elif pos == 'F1':
            #set direction to forward
            #move forward until hit F1
            #stop
            if self.x_axis_set_forward():
                #F2 is included in check for safety redundancy
                while self.check_for_positive_pins(['F1','F2']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F0 to ', pos)
                #set E0 to low
                writeMotorsDIO('E0',0)
                return True
            else:
                writeMotorsDIO('E0',0)
                print("Error: X axis did not travel to ", pos)
                return False
        elif pos == 'F2':
            #set direction to forward
            #move until you hit F2 and stop
            if self.x_axis_set_forward():
                while self.check_for_positive_pins(['F2']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F0 to ', pos)
                #set E0 to low
                writeMotorsDIO('E0',0)
                return True
            else:
                print("Error: X axis did not travel to ", pos)
                writeMotorsDIO('E0',0)                    
                return False
    elif self.check_for_positive_pins(['F1']):
        #print('x_goto at F1')
        if pos == 'F0':
            #set direction to backward
            #move back until F0
            #stop
            if self.x_axis_set_backwards():
                #until you hit F0, move backwards
                #included F2 for safety
                while self.check_for_positive_pins(['F0','F2']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F1 to ', pos)
                #turn motor off
                writeMotorsDIO('E0',0)
                return True
            else:
                writeMotorsDIO('E0',0)
                print("Error: X axis did not travel to ", pos)
                return False
        elif pos == 'F1':
            return True
        elif pos == 'F2':
            #set direction to forward
            #move forward
            #stop when hit F2
            if self.x_axis_set_forward():
                #until you hit F0, move backwards
                while self.check_for_positive_pins(['F0','F2']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F1 to ', pos)
                #turn motor off
                writeMotorsDIO('E0',0)
                return True
            else:
                writeMotorsDIO('E0',0)
                print("Error: X axis did not travel to ", pos)
                return False
    elif self.check_for_positive_pins(['F2']):
        if pos == 'F0':
            #set direction to backwards
            #move backwards until hit F0
            #stop
            if self.x_axis_set_backwards():
                #until you hit F0, move backwards
                #included F2 for safety
                while self.check_for_positive_pins(['F0']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F2 to ', pos)
                #turn motor off
                writeMotorsDIO('E0',0)
                return True
            else:
                writeMotorsDIO('E0',0)
                print("Error: X axis did not travel to ", pos)
                return False
        elif pos == 'F1':
            #set direction to backwards
            #move backwards until hit F1
            #stop
            if self.x_axis_set_backwards() == True:
                #until you hit F0, move backwards
                #included F0 for Safety
                while self.check_for_positive_pins(['F0','F1']) == False:
                    #set E0 to high (x-motor go)
                    writeMotorsDIO('E0',1)
                print('X axis traveled from F2 to ', pos)
                #turn motor off
                writeMotorsDIO('E0',0)
                return True
            else:
                writeMotorsDIO('E0',0)
                print("Error: X axis did not travel to ", pos)
                return False
        elif pos == 'F2':
            return True
    else:
        #if in the null position
        #move foward until you hit a switch
        #stop
        #jump back into go_to position
        self.x_axis_find_switch()
        self.x_axis_go_to(pos)

def x_axis_set_forward(self):
    #check that you're not at F2
    #set direction forward
    if self.check_for_positive_pins(['F2']):
        print("Danger Will Robinson: X axis already at F2. Do not set motors forward!")
        writeDIO('E1',0)
        sys.exit(0)
        return False
    else:
        writeDIO('E1',1)
    return True

def x_axis_set_backwards(self):
    #check that you're not at F0 (home)
    #set direction backwards
    if self.check_for_positive_pins(['F0']):
        print("Danger Will Robinson: X axis already at F0. Do not set motors backwards!")
        writeDIO('E1',1)
        sys.exit(0)
        return False
    else:
        writeDIO('E1',0)
    return True
