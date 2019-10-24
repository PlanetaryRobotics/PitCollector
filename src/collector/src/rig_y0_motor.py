from tools.labjack_tools import *

import rospy
import time
import numpy as np

def y0_axis_go_to(self, pos):
    #where am I at?
    #which pin am I going to?
    #what direction is this pin?
    #the y0 axis has two pin F4,F5

    if pos in ['F4','F5']:
        pass
    else:
        print('y0_axis_go_to: invalid go to pin')
        sys.exit(0)
        return False

    #if you're at the bottom
    #print('y0_axis_go_to check if F4')
    if self.check_for_positive_pins(['F4']):
        if pos == 'F4':
            return True
        elif pos == 'F5':
            #set direction to up
            #move until you hit F5
            #stop
            if self.y0_axis_set_up() == True:
                #run the full motor for a second
                #then switch to the small increment motor movements until you hit the switch
                #this is to protect the y0 lift from crushing itself
                #it moves VERY fast and there's a brief delay with the mechnical switch that could break things
                #move for 8 seconds, then slowly in 3 second increments
                #print('y0_axis_go_to: move for fixed time')
                if self.y0_move_for_fixed_time(['F5'],16):
                    print('y0 axis traveled from F4 to ', pos)
                    return True
                time.sleep(.5)
                for i in range(1,10):
                    if self.y0_move_for_fixed_time(['F5'],3):
                        print('y0 axis traveled from F4 to ', pos)
                        return True
                    print('sleep')
                    time.sleep(1)
            else:
                writeMotorsDIO('E2',0)
                print("y0_axis_go_to: error...y0 axis did not travel to ", pos)
                return False
    #if you're at the top already

    elif self.check_for_positive_pins(['F5']):
        #print('y0_axis_go_to check if F5')
        if pos == 'F4':
            #print ( self.y0_axis_set_down() )
            if self.y0_axis_set_down() == True:
                #move for 8 seconds, then slowly in 3 second increments
                #print('y0_axis_go_to: move for fixed time')
                if self.y0_move_for_fixed_time(['F4'],16):
                    print('y0 axis traveled from F5 to ', pos)
                    return True
                time.sleep(.5)
                for i in range(1,10):
                    if self.y0_move_for_fixed_time(['F4'],3):
                        print('y0 axis traveled from F5 to ', pos)
                        return True
                    print('sleep')
                    time.sleep(1)
            else:
                writeMotorsDIO('E2',0)
                print("y0_axis_go_to: error...y0 axis did not travel to ", pos)
                sys.exit(0)
                return False
        elif pos == 'F5':
            return True
    else:
        self.y0_axis_find_switch()
        self.y0_axis_go_to(pos)

def y0_move_for_fixed_time(self,pos,time_length):
    #this function briefly moves and stops the y0 lift motor
    #the y0 lift closes very fast towards the end

    #safefty checks
    for i in pos:
        if i in ['F4','F5']:
            pass
        else:
            print('y0_move_for_fixed_time: invalid go to pin')
            writeMotorsDIO('E2',0)
            sys.exit(0)
            return False

    if self.check_for_positive_pins(pos) == True:
        print('y0_move_for_fixed_time: switches already triggered. exiting')
        return True
    if time_length > 0 and time_length < 100:
        pass
    else:
        print('y0_move_for_fixed_time: time length invalid', time_length)
        return False

    #run full for x seconds or until you hit a switch
    start_time = time.time()
    elapsed_time = 0.00
    while self.check_for_positive_pins(pos) == False and elapsed_time < time_length:
        #turn on motor
        writeMotorsDIO('E2',1)
        elapsed_time = time.time() - start_time
        print(elapsed_time)
        #double check for safety did you hit the switch
        if self.check_for_positive_pins(pos):
            print('y0 axis traveled from to ', pos)
            writeMotorsDIO('E2',0)
            break

    #turn motor off again for safety
    writeMotorsDIO('E2',0)


def y0_axis_find_switch(self):

    ans = str(raw_input('No switch active on y0 (lower lift) axis...move UP until you hit a switch? (y/n):'))
    if ans in ['yes', 'y']:
        for i in range(1,10):
            writeDIO('E3',1)
            if self.check_for_positive_pins(['F4','F5']):
                print('y0_axis_find_switch: switch triggered')
                break
            else:
                if self.y0_move_for_fixed_time(['F4','F5'],3):
                    break
                time.sleep(.5)


    elif ans in ['no', 'n']:
        ans2 = str(raw_input('move y0 (lower lift) axis DOWN until you hit a switch? (y/n):'))
        if ans2 in ['yes','y']:
            for i in range(1,10):
                #set motor to backwards until you hit a positive switch
                writeDIO('E3',0)
                if self.check_for_positive_pins(['F4','F5']):
                    print('y0_axis_find_switch: switch triggered')
                    return True
                    break
                else:
                    if self.y0_move_for_fixed_time(['F4','F5'],3):
                        return True
                        break
                    time.sleep(.5)

        else:
            print("y0_axis_find_switch: aborting")
            sys.exit(0)
            return False
    else:
        print("y0_axis_find_switch: aborting")
        sys.exit(0)
        return False

def y0_axis_set_up(self):
    #check that you're not at the top F5
    #change direction to up
    if self.check_for_positive_pins(['F5']) == True:
        print("Danger Will Robinson!! Y0 axis already at top, F5. Do not set motors up!")
        writeDIO('E3',0)
        return False

    writeDIO('E3',1)
    return True

def y0_axis_set_down(self):
    #check that you're not at the bottom F4
    #change direction to down
    #print('y0_axis_set_down')
    if self.check_for_positive_pins(['F4']) == True:
        print("Danger Will Robinson!! Y0 axis already at bottom, F4. Do not set motors down!")
        writeDIO('E3',1)
        sys.exit(0)
        return False

    #print('y0_set_down: true')
    writeDIO('E3',0)
    return True
