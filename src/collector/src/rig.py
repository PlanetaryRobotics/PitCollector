from tools.labjack_tools import *
from tools.camera_tools import *
from tools.pan_tilt_tools import *

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class Rig:
    def __init__(self):
        self.pos = 0
        self.go_home()

        self.pan = 0
        self.tilt = 0
        self.set_pan_tilt(self.pan, self.tilt)
        
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.camera_topic = ''
        self.file_path = ''
        self.image_count = 0
        self.step_size = 0
        self.base_grey = 0
        self.hdr = False

    def imu_callback(self, msg):
        quat = msg.orientation
        r,p,y = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        alpha = 0.99
        beta = 1-alpha
        self.roll = self.roll*alpha + beta*np.degrees(r)
        self.pitch = self.pitch*alpha + beta*np.degrees(p)
        self.yaw = self.yaw*alpha + beta*np.degrees(y)
        print(self.roll, self.pitch)

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

    def take_and_save_images(self,camera_topic,file_path,image_count,step_size,base_grey,hdr):
        self.camera_topic = str(camera_topic)
        self.file_path = str(file_path)
        self.image_count = int(image_count)
        self.step_size = int(step_size)
        self.base_grey = int(base_grey)
        self.hdr = bool(hdr)

        takeAndSaveImages(self, self.camera_topic,self.file_path,self.image_count,self.step_size,self.base_grey,self.hdr)
        return 

    def update(self):
        pass
