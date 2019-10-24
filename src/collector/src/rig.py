from tools.labjack_tools import *
from tools.pan_tilt_tools import *

import rig_x_motor
import rig_y1_motor
import rig_y0_motor

import rospy
import time
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import json

from camera import Camera

class Rig:
    def __init__(self):
        self.pos = 0
        self.go_home()

        self.pan = 0
        self.tilt = 0
        #temporarily turns off for my sanity
        #self.set_pan_tilt(self.pan, self.tilt)
        
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.positionMap = { \
        '7': ['F0','F5','F7'], '8':['F1','F5','F7'], '9':['F2','F5','F7'], \
        '4': ['F0','F5','F6'], '5':['F1','F5','F6'], '6':['F2','F5','F6'],  \
        '1': ['F0','F4','F6'], '2':['F1','F4','F6'], '3':['F2','F4','F6'],  \
        }

        #needed to create a camera object in the rig b/c the camera is triggered from inside the rig's logic
        self.camera = Camera()

        self.currentPosition = ''
        #self.currentPosition = self.get_current_position()
        #print (self.currentPosition)

        #this initilizes the positive pins list
        self.check_for_positive_pins(['F0'])

    def imu_callback(self, msg):
        quat = msg.orientation
        r,p,y = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        alpha = 0.99
        beta = 1-alpha
        self.roll = self.roll*alpha + beta*np.degrees(r)
        self.pitch = self.pitch*alpha + beta*np.degrees(p)
        self.yaw = self.yaw*alpha + beta*np.degrees(y)

    def set_pan_tilt(self, pan, tilt):
        self.pan = pan
        self.tilt = tilt
        setPanTilt(self.pan, self.tilt)

    def go_home(self):
        self.pos = 0

    def go_to_next_pos(self):
        print("Going to position {}.".format(pos))
        self.pos += 1 

    def get_roll_pitch(self):
        return self.roll, self.pitch

    def get_current_position(self):
        # first read all the F pins (limit switches)
        # second convert the positive pins to a list of their names (this is to keep the code readable)
        # third convert the list of positive pins into a position
        # note - I kept the pins as names instead of arrays, to make the output more readable
        values = self.read_all_input_pins()
        print(values)
        true_names = self.return_true_pin_names(values)
        print(true_names)
        self.currentPosition = self.convert_pins_to_position_names(true_names)
        return self.currentPosition

    def read_all_input_pins(self):
        input_pin_names = ['F0','F1','F2','F3','F4','F5','F6','F7']
        values = []
        #read all input pins
        for i in input_pin_names:
            #F3 is not wired to anything. But I need to keep it in the list to make the numbers easy to read 0-7
            if i != 'F3':
                values.append(readDIO(i))
            else:
                values.append(False)
        return values

    def return_true_pin_names(self, values):
        positive_pin_names = []
        #loop through pins F0 to F7, create a list of the pin names that are positive
        z=0
        for i in values:
            if i:
                #only the F pins give position so we're only using an F prefix
                positive_pin_names.append('F' + str(z))
            z += 1
        return positive_pin_names

    def convert_pins_to_position_names(self,true_pin_names):
        #print(self.positionMap)
        for position in self.positionMap.keys():
            #if no pins are on, or pin names are invalid...
            if true_pin_names != [] and self.positionMap != {}:
                if self.positionMap[position].sort() & true_pin_names.sort():
                    return position
                else:
                    print('Failed to convert positive pin names to position names') 
            else:
                return 'null_position'
        return -1

    def check_for_positive_pins(self,values):
        #send a list of positive pin names
        #return true if any on them are fired
        #added this input protection because this function HAS to be right or the machine wanders off
        for i in values:
            if i in ['F0','F1','F2','F3','F4','F5','F6','F7']:
                pass
            else:
                #print('check_for_positive_pins: invalid pin input. aborting.')
                sys.exit(0)
                return False

        positive_pins = self.return_true_pin_names(self.read_all_input_pins())
        #print('check_for_positive_pins:',positive_pins)
        #this is an OR check. If either pin is in the positive pin list, exit the function and return true
        #this makes is easy to set two limits
        #print(values)
        for i in set(values):
            #print(i,positive_pins)
            if i in set(positive_pins):
                print('check_for_positive_pins: triggered.', i)
                return True
            else:
                #print('check_for_positive_pins: failed', i)
                pass
        return False

    def go_to_pos(self,pos):
        if pos in range(1,9):
            self.x_axis_go_to(self.positionMap[str(pos)][0])
            self.y0_axis_go_to(self.positionMap[str(pos)][1])        
            self.y1_axis_go_to(self.positionMap[str(pos)][2])
            print('Rig: successfully moved to position', pos)

    def go_home_no_safeguard(self):
        self.go_to_pos(1)
        self.set_pan_tilt(0,0)

    def run_full_sequence(self,filepath):
        print('rig: loading json position sequence')
        with open(filepath) as json_file:
            data = json.load(json_file)
        #for each position

        cnt = 0
        for position_pan_tilt in data['position_pan_tilt']:
            #goto first position in json
            if cnt == 0:
                position = position_pan_tilt[0]
                pan  = position_pan_tilt[1]
                tilt = position_pan_tilt[2]
                #goto first position
                print('rig: initialize position',position)
                self.go_to_pos(position)
                #goto pan/tilt
                print('rig: initialize pan/tilt',pan,tilt )
                self.set_pan_tilt(pan,tilt)

            elif position != position_pan_tilt[0]:
                position = position_pan_tilt[0]
                #goto new position
                print('rig: change position',position)
                self.go_to_pos(position)

            if pan != position_pan_tilt[1] or tilt != position_pan_tilt[2]:
                #goto next pan tilt
                pan  = position_pan_tilt[1]
                tilt = position_pan_tilt[2]
                #goto pan/tilt
                print('rig: change pan/tilt',pan,tilt )
                self.set_pan_tilt(pan,tilt)
            
            print('rig:taking 3 bracketed images - position number',cnt)
            resp = self.camera.take_3_bracketed_images('/camera/image_color','../PitCollector/data',3,20000)

            cnt += 1
            #end of loop

        #end loop, close program
        self.go_home_no_safeguard()
        print('rig:sequence compelete')

    def kill_all_motors(self):
        #function to shut it all off
        writeMotorsDIO('E0',0)
        writeMotorsDIO('E2',0)
        writeMotorsDIO('E4',0)
        print('kill_all_motors: success')
        return True

    def update(self):
        pass

    def x_axis_find_switch(self):
        return rig_x_motor.x_axis_find_switch(self)

    def x_axis_go_to(self, pos):
        return rig_x_motor.x_axis_go_to(self,pos)

    def x_axis_set_forward(self):
        return rig_x_motor.x_axis_set_forward(self)

    def x_axis_set_backwards(self):
        return rig_x_motor.x_axis_set_backwards(self)

    def y0_axis_go_to(self, pos):
        return rig_y0_motor.y0_axis_go_to(self,pos)

    def y0_move_for_fixed_time(self,pos,time_length):
        return rig_y0_motor.y0_move_for_fixed_time(self,pos,time_length)

    def y0_axis_find_switch(self):
        return rig_y0_motor.y0_axis_find_switch(self)

    def y0_axis_set_up(self):
        return rig_y0_motor.y0_axis_set_up(self)

    def y0_axis_set_down(self):
        return rig_y0_motor.y0_axis_set_down(self)

    ##y1 functions
    def y1_axis_go_to(self, pos):
        return rig_y1_motor.y1_axis_go_to(self,pos)

    def y1_move_for_fixed_time(self,pos,time_length):
        return rig_y1_motor.y1_move_for_fixed_time(self,pos,time_length)

    def y1_axis_find_switch(self):
        return rig_y1_motor.y1_axis_find_switch(self)

    def y1_axis_set_up(self):
        return rig_y1_motor.y1_axis_set_up(self)

    def y1_axis_set_down(self):
        return rig_y1_motor.y1_axis_set_down(self)
    