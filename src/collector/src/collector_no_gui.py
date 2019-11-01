#! /usr/bin/env python
import rospy
import numpy as np

from rig import Rig

import time
import json

#includes for grabbing image from topic stream and outputting it to GUI
#https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import cv2
import dynamic_reconfigure.client
import rospy

from std_msgs.msg import Float32


class Collector:
    def __init__(self):
        self.rig = Rig()
        #the switch can be left on from previoius hanging run. need to kill it ASAP
        self.rig.kill_all_motors()

        #https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/
        rospy.Subscriber("/camera/image_color", ImageMSG, self.image_callback)
        self.bridge = CvBridge()
        self.image_stream_count = 0

        self.current_exposure = -1
        rospy.Subscriber("/camera/exposure_time", Float32, self.exposure_callback)

        self.rig.kill_all_motors()

        #to resolve conflict with GUI and image callback, lets create a local copy
        #https://stackoverflow.com/questions/16235955/create-a-multichannel-zeros-mat-in-python-with-cv2/16236373
        self.cv_mat = np.zeros((3248, 4096, 3), dtype = "uint8")

    def exposure_callback(self,msg):
        self.current_exposure = msg.data

    def image_callback(self, msg):
        #print('collector: in gui image_callback')
        try:
            # grab every nth image
            if self.image_stream_count % 5 == 0 :
                #print("collector: streaming received image from image_raw")
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                #shrink the camera ouput for the GUI
                cv2_img_small = cv2.resize(cv2_img,(500,500))
                if cv2_img_small is not None:
                    pass
                    #print('collector: ROS image converted for GUI')
                if cv2.imwrite('../PitCollector/GUI.jpeg', cv2_img_small):
                    pass
                    #print('collector: GUI image saved')

                #img = cv2.imread('/tmp/1.jpg')
                #print cv2_img.shape, cv2_img.dtype
        except CvBridgeError, e:
            print(e)
        else:
            pass
        self.image_stream_count += 1

    def update(self):
        if not rospy.is_shutdown():
            pass
        else:
            self.root.destroy()

    def update_roll_pitch_labels(self):
        roll, pitch = self.rig.get_roll_pitch()

    #https://stackoverflow.com/questions/295135/turn-a-string-into-a-valid-filename
    def slugify(self,value):
        """
        Normalizes string, converts to lowercase, removes non-alpha characters,
        and converts spaces to hyphens.
        """
        import unicodedata
        import re
        #value = unicodedata.normalize('NFKD', value).encode('ascii', 'ignore')
        value = unicode(re.sub('[^\w\s-]', '', value).strip().lower())
        value = unicode(re.sub('[-\s]+', '-', value))
        return value

######################################################3
    def main(self):

        self.rig.kill_all_motors()
        while not rospy.is_shutdown():
            #if rig is out of position, get it into position before doing anything else
            #all code logic depends on rig being in a discrete position 
            if self.rig.get_current_position() == -1:
                print('collector: rig is out of position. It needs to be in a discrete position before doing anything.')
                print('collector: finding all limit switches')
                self.rig.x_axis_find_switch()
                self.rig.y0_axis_find_switch()
                self.rig.y1_axis_find_switch()
                print('collector: going home with safeguards')
                self.rig.go_home_with_safeguards()

            print('')
            print('************************************')
            print('**********PitCollector**************')
            print('Choose your option (type the number 1,2,3,4..):')
            print("1 - run the exposure test")
            print("2 - run the full sequence")
            print("3 - run a checkout sequence")
            print("4 - run short_test.json for debugging")
            print("5 - go to home position without scraping the ground")
            print("6 - read the roll pitch")
            print("7 - quit and kill the program")
            val = input("Enter your option: ")
            if val == 1:
                print('')
                data_folder = str(raw_input("What do you want to name your folder (example: exposure_test): "))
                print("Running exposure test:")
                #clean up the folder name before saving it
                data_filepath = '/home/pipedream/PitCollector/data/' + self.slugify(data_folder)
                print('Will save to',data_filepath)
                self.rig.run_full_sequence('/camera/image_color','/home/pipedream/PitCollector/json_sequences/exposure_test.json',data_filepath)
                print('collector: going home with safeguards')
                self.rig.go_home_with_safeguards()
            elif val == 2:
                print('')
                data_folder = str(raw_input("What do you want to name your folder (example: location_1): "))
                print("Running full sequence:")
                #clean up the folder name before saving it
                data_filepath = '/home/pipedream/PitCollector/data/' + self.slugify(data_folder)
                print('Will save to',data_filepath)
                self.rig.run_full_sequence('/camera/image_color','/home/pipedream/PitCollector/json_sequences/full_sequence.json',data_filepath)
                print('collector: going home with safeguards')
                self.rig.go_home_with_safeguards()
            elif val == 3:
                print('')
                data_folder = str(raw_input("What do you want to name your folder (example: checkout_sequence): "))
                print("Running checkout sequence:")
                #clean up the folder name before saving it
                data_filepath = '/home/pipedream/PitCollector/data/' + self.slugify(data_folder)
                print('Will save to',data_filepath)
                self.rig.run_full_sequence('/camera/image_color','/home/pipedream/PitCollector/json_sequences/checkout_sequence.json',data_filepath)
                print('collector: going home with safeguards')
                self.rig.go_home_with_safeguards()
            elif val == 4:
                print('')
                data_folder = str(raw_input("What do you want to name your folder (example: checkout_sequence): "))
                print("Running debug test sequence:")
                #clean up the folder name before saving it
                data_filepath = '/home/pipedream/PitCollector/data/' + self.slugify(data_folder)
                print('Will save to',data_filepath)
                self.rig.run_full_sequence('/camera/image_color','/home/pipedream/PitCollector/json_sequences/short_test.json',data_filepath)
                print('collector: going home with safeguards')
                self.rig.go_home_with_safeguards()
            elif val == 5:
                print('')
                print('Going to home position with safeguards for scraping the ground')
                self.rig.go_home_with_safeguards()
            elif val == 6:
                print('roll,pitch',self.rig.get_roll_pitch())
            elif val == 7:
                sys.exit(0)
            else:
                print('Invalid input')
                print('')

######################################################

def kill_ros():
    import subprocess
    subprocess.call(["rosnode", "kill", "--all"])
    subprocess.call(["killall", "-9", "rosmaster"])
    subprocess.call(["killall", "-9", "roscore"])

if __name__ == "__main__":
    rospy.init_node('collector_no_gui')
    gui = Collector()
    gui.main()
    if not rospy.is_shutdown():
        #kill all motors in event of abort
        gui.rig.kill_all_motors()
        kill_ros()
