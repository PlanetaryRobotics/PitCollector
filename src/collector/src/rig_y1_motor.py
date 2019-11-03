from tools.labjack_tools import *

import rospy
import time
import numpy as np

def y1_axis_go_to(self, pos):
    #where am I at?
    #which pin am I going to?
    #what direction is this pin?
    #the y1 axis has two pin F6,F7

    if pos in ['F6','F7']:
        pass
    else:
        print('y1_axis_go_to: invalid go to pin')
        sys.exit(0)
        return False

    #if you're at the bottom
    #print('y1_axis_go_to check if F6')
    if self.check_for_positive_pins(['F6']):
        if pos == 'F6':
            return True
        elif pos == 'F7':
            #set direction to up
            #move until you hit F7
            #stop
            if self.y1_axis_set_up() == True:
                #run the full motor for a second
                #then switch to the small increment motor movements until you hit the switch
                #this is to protect the y1 lift from crushing itself
                #it moves VERY fast and there's a brief delay with the mechnical switch that could break things
                #move for 8 seconds, then slowly in 3 second increments
                #print('y1_axis_go_to: move for fixed time')
                if self.y1_move_for_fixed_time(['F7'],120):
                    print('y1 axis traveled from F6 to ', pos)
                    return True
                time.sleep(.5)
                for i in range(1,10):
                    if self.y1_move_for_fixed_time(['F7'],120):
                        print('y1 axis traveled from F6 to ', pos)
                        return True
                    #print('sleep')
                    time.sleep(.5)
            else:
                writeMotorsDIO('E4',0)
                print("y1_axis_go_to: error...y1 axis did not travel to ", pos)
                return False
    #if you're at the top already

    elif self.check_for_positive_pins(['F7']):
        #print('y1_axis_go_to check if F7')
        if pos == 'F6':
            #print ( self.y1_axis_set_down() )
            if self.y1_axis_set_down() == True:
                #move for 8 seconds, then slowly in 3 second increments
                #print('y1_axis_go_to: move for fixed time')
                if self.y1_move_for_fixed_time(['F6'],120):
                    print('y1 axis traveled from F7 to ', pos)
                    return True
                time.sleep(.5)
                for i in range(1,10):
                    if self.y1_move_for_fixed_time(['F6'],120):
                        print('y1 axis traveled from F7 to ', pos)
                        return True
                    #print('sleep')
                    time.sleep(1)
            else:
                writeMotorsDIO('E4',0)
                print("y1_axis_go_to: error...y1 axis did not travel to ", pos)
                sys.exit(0)
                return False
        elif pos == 'F7':
            return True
    else:
        self.y1_axis_find_switch()
        self.y1_axis_go_to(pos)

def y1_move_for_fixed_time(self,pos,time_length):
    #this function briefly moves and stops the y1 lift motor
    #the y1 lift closes very fast towards the end

    #safefty checks
    for i in pos:
        if i in ['F6','F7']:
            pass
        else:
            print('y1_move_for_fixed_time: invalid go to pin')
            writeMotorsDIO('E4',0)
            sys.exit(0)
            return False

    if self.check_for_positive_pins(pos) == True:
        print('y1_move_for_fixed_time: switches already triggered. exiting')
        return True
    if time_length > 0 and time_length < 500:
        pass
    else:
        print('y1_move_for_fixed_time: time length invalid', time_length)
        return False

    #run full for x seconds or until you hit a switch
    start_time = time.time()
    elapsed_time = 0.00
    while self.check_for_positive_pins(pos) == False and elapsed_time < time_length:
        #turn on motor
        writeMotorsDIO('E4',1)
        elapsed_time = time.time() - start_time
        #print(elapsed_time)
        #double check for safety did you hit the switch
        if self.check_for_positive_pins(pos):
            print('y1 axis traveled from to ', pos)
            writeMotorsDIO('E4',0)
            break

    #turn motor off again for safety
    writeMotorsDIO('E4',0)

def y1_axis_find_switch(self):

    ans = str(raw_input('No switch active on y1 (upper lift) axis...move UP until you hit a switch? (y/n):'))
    if ans in ['yes', 'y']:
        for i in range(1,10):
            writeDIO('E5',1)
            if self.check_for_positive_pins(['F6','F7']):
                print('y1_axis_find_switch: switch triggered')
                break
            else:
                if self.y1_move_for_fixed_time(['F6','F7'],120):
                    break
                time.sleep(.5)


    elif ans in ['no', 'n']:
        ans2 = str(raw_input('move y1 (upper lift) axis DOWN until you hit a switch? (y/n):'))
        if ans2 in ['yes','y']:
            for i in range(1,10):
                #set motor to backwards until you hit a positive switch
                writeDIO('E5',0)
                if self.check_for_positive_pins(['F6','F7']):
                    print('y1_axis_find_switch: switch triggered')
                    return True
                    break
                else:
                    if self.y1_move_for_fixed_time(['F6','F7'],120):
                        return True
                        break
                    time.sleep(.5)

        else:
            print("y1_axis_find_switch: aborting")
            sys.exit(0)
            return False
    else:
        print("y1_axis_find_switch: aborting")
        sys.exit(0)
        return False

def y1_axis_set_up(self):
    #check that you're not at the top F7
    #change direction to up
    if self.check_for_positive_pins(['F7']) == True:
        print("Danger Will Robinson!! y1 axis already at top, F7. Do not set motors up!")
        writeDIO('E5',0)
        return False

    writeDIO('E5',1)
    return True

def y1_axis_set_down(self):
    #check that you're not at the bottom F6
    #change direction to down
    #print('y1_axis_set_down')
    if self.check_for_positive_pins(['F6']) == True:
        print("Danger Will Robinson!! y1 axis already at bottom, F6. Do not set motors down!")
        writeDIO('E5',1)
        sys.exit(0)
        return False

    #print('y1_set_down: true')
    writeDIO('E5',0)
    return True
