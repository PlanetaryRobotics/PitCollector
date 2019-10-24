from tools.camera_tools import *

import rospy

class Camera:
    def __init__(self):
        self.camera_topic = ''
        self.file_path = ''
        self.image_count = 0
        self.exposure_time_microseconds = 0

    def take_3_bracketed_images(self,camera_topic,file_path,image_count,exposure_time_microseconds):
        #file name should be ../PitCollector/data instead of /home/PitCollector/data
        #the module refers to the folders in reference to it's current working directory
        print('camera.py: taking 3 bracketed images')
        self.camera_topic = str(camera_topic)
        self.file_path = str(file_path)
        self.image_count = int(image_count)
        self.exposure_time_microseconds = int(exposure_time_microseconds)
        resp = tools_take_3_bracketed_images(self.camera_topic,self.file_path,self.image_count,self.exposure_time_microseconds)
        return resp

    '''
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
    '''
    def update(self):
        pass
