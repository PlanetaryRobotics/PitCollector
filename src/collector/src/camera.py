from tools.camera_tools import *

import rospy

class Camera:
    def __init__(self):
        self.camera_topic = ''
        self.file_path = ''
        self.image_count = 0
        self.step_size = 0
        self.base_grey = 0
        self.hdr = False

    def take_and_save_images(self,camera_topic,file_path,image_count,step_size,base_grey,hdr):
        #file name should be ../PitCollector/data instead of /home/PitCollector/data
        #the module refers to the folders in reference to it's current working directory
        self.camera_topic = str(camera_topic)
        self.file_path = str(file_path)
        self.image_count = int(image_count)
        self.step_size = int(step_size)
        self.base_grey = int(base_grey)
        self.hdr = bool(hdr)

        resp = takeAndSaveImages(self.camera_topic,self.file_path,self.image_count,self.step_size,self.base_grey,self.hdr)
        return resp

    def update(self):
        pass
