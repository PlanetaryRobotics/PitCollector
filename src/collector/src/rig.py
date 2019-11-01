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
        #this is for safety
        self.kill_all_motors()
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

        self.positionMap_alternate_config = { \
        '7': ['F0','F5','F7'], '8':['F1','F5','F7'], '9':['F2','F5','F7'], \
        '4': ['F0','F4','F7'], '5':['F1','F4','F7'], '6':['F2','F4','F7'],  \
        '1': ['F0','F4','F6'], '2':['F1','F4','F6'], '3':['F2','F4','F6'],  \
        }

        #needed to create a camera object in the rig b/c the camera is triggered from inside the rig's logic
        self.camera = Camera()
        #print (self.currentPosition)

        #this initilizes the positive pins list which will be blank and cause problems if you don't run it before using it
        self.check_for_positive_pins(['F0'])
        self.check_for_positive_pins(['F0'])
        #run this after check_for_positive_pins because check_for_positive_pins needs to initialize first
        #and this function depends on it
        self.currentPosition = ''
        self.currentPosition = self.get_current_position()
        print('rig: current position is', self.currentPosition)

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

    def get_roll_pitch(self):
        return self.roll, self.pitch

    def get_current_position(self):
        # first read all the F pins (limit switches)
        # second convert the positive pins to a list of their names (this is to keep the code readable)
        # third convert the list of positive pins into a position
        # note - I kept the pins as names instead of arrays, to make the output more readable
        values = self.read_all_input_pins()
        #print(values)
        true_names = self.return_true_pin_names(values)
        #print(true_names)
        self.currentPosition = self.convert_pins_to_position_names(true_names)
        if self.currentPosition not in ['1','2','3','4','5','6','7','8','9']:
            return -1
        else:
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
        #you need 3 switches to calculate position
        if len(true_pin_names) < 3:
            print('insufficient switches triggered to calculate position.')
            return -1

        for position in self.positionMap.keys():
            #if no pins are on, or pin names are invalid...
            if true_pin_names is not None:
                #print (true_pin_names)
                #https://www.geeksforgeeks.org/python-check-if-two-lists-are-identical/
                print(true_pin_names[0], true_pin_names[1], true_pin_names[2])
                print(self.positionMap[position])
                if true_pin_names[0] in self.positionMap[position] and true_pin_names[1] in self.positionMap[position] and true_pin_names[2] in self.positionMap[position]:
                    #print('convert_pin triggers true')
                    #print(true_pin_names)
                    #print(self.positionMap[position])
                    #print('Matches position:', position)
                    return position
                elif true_pin_names[0] in self.positionMap_alternate_config[position] and true_pin_names[1] in self.positionMap_alternate_config[position] and true_pin_names[2] in self.positionMap_alternate_config[position]:
                    #if you find yourself on the 2nd level with the lower lift down
                    #and the upper lift up, you need to drop the upper lift and
                    #raise the lower lift to get into the pose that this program expects
                    print('rig: youre in an unexpected pose. correcting now')
                    #raise the lower lift up
                    self.y0_axis_go_to(['F5'])
                    #drop the upper lift down
                    self.y1_axis_go_to(['F6'])
                    print('rig: returned pose to expected position')
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
        #position numbers are between 1 and 9
        # 
        # 7 8 9
        # 4 5 6
        # 1 2 3
        #
        # position map
        if pos in range(1,10):
            current_pos = int(self.get_current_position()) 

            #protection against bottom scraping
            if current_pos == 1 and pos in [2,3]:
                ans = str(raw_input("WARNING You're going to scrape on the bottom. Abort? (y/n) "))
                if ans == 'y':
                    sys.exit(0)
                else:
                    pass
            elif current_pos == 2 and pos in [1,3]:
                ans = str(raw_input("WARNING You're going to scrape on the bottom. Abort? (y/n) "))
                if ans == 'y':
                    sys.exit(0)
                else:
                    pass
            elif current_pos == 3 and pos in [1,2]:
                ans = str(raw_input("WARNING You're going to scrape on the bottom. Abort? (y/n) "))
                if ans == 'y':
                    sys.exit(0)
                else:
                    pass
            self.x_axis_go_to(self.positionMap[str(pos)][0])
            self.y0_axis_go_to(self.positionMap[str(pos)][1])        
            self.y1_axis_go_to(self.positionMap[str(pos)][2])
            print('Rig: successfully moved to position', pos)
            #refresh current position
            self.currentPosition = self.get_current_position()

    def go_home_no_safeguard(self):
        self.go_to_pos(1)
        self.set_pan_tilt(0,0)

    def go_home_with_safeguards(self):
        self.set_pan_tilt(0,0)
        #read current position
        self.currentPosition = self.get_current_position()
        # 
        # 7 8 9
        # 4 5 6
        # 1 2 3
        #
        # position map
        # let's prevent scraping on the floor when at position 1,2, or 3
        if self.currentPosition in ['4','5','6']:
            self.go_to_pos(4)
            self.go_to_pos(1)
        elif self.currentPosition in ['7','8','9']:
            self.go_to_pos(7)
            self.go_to_pos(1)
        elif self.currentPosition == '2':
            self.go_to_pos(5)
            self.go_to_pos(4)
            self.go_to_pos(1)
        elif self.currentPosition == '3':
            self.go_to_pos(6)
            self.go_to_pos(4)
            self.go_to_pos(1)
        elif self.currentPosition == '1':
            pass
        else:
            print('go_home_with_safeguards: invalid currentPosition')
            sys.exit(0)


    def run_full_sequence(self,camera_topic, json_filepath,data_filepath):
        print('rig: loading json position sequence')
        print(self.get_current_position())
        with open(json_filepath) as json_file:
            data = json.load(json_file)

        cnt = 0
        #added for path optimization
        #you don't want to take images at places you've already been
        take_images = True
        position_set = []
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
                print('rig: initialize pan/tilt',pan,tilt)
                self.set_pan_tilt(pan,tilt)

                #add the first position to the set
                position_set.append(position)
            elif position != position_pan_tilt[0]:
                position = position_pan_tilt[0]
                #when you move to a new position, add it to the position set
                #if you've already been here at this position, don't take images
                if position in position_set:
                    take_images = False
                    print('Rig: skipping images because weve been here before.')
                else:
                    position_set.append(position)
                    take_images = True
                #goto new position
                print('rig: change position',position)
                self.go_to_pos(position)

            #if the pan/tilt hasn't changed since last position, don't change it
            if pan != position_pan_tilt[1] or tilt != position_pan_tilt[2]:
                #goto next pan tilt
                pan  = position_pan_tilt[1]
                tilt = position_pan_tilt[2]
                #goto pan/tilt
                print('rig: change pan/tilt',pan,tilt )
                self.set_pan_tilt(pan,tilt)
            
            #pull the image_count and exposure baseline from the json file
            print('rig current position before images',self.currentPosition)
            if take_images == True:
                image_count =  data["number_of_bracketed_images"] 
                exposure_time_microseconds = data["baseline_exposure_microseconds"] 
                bracketing_factor =  data["bracketing_factor"] 
                print('rig:taking bracketed images at loop number', cnt)
                resp = self.camera.take_n_bracketed_images(camera_topic,data_filepath,image_count, \
                    exposure_time_microseconds,bracketing_factor, \
                    self.currentPosition,self.pan,self.tilt)
            cnt += 1
            #end of loop
        print('rig: sequence compelete')

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
    