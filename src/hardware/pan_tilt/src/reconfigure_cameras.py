#!/usr/bin/env python

PACKAGE = 'pointgrey_camera_driver'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import pdb

import dynamic_reconfigure.client

class Config:

    def __init__(self,topic,defaults):
        self.topic = topic
        self.client = dynamic_reconfigure.client.Client(topic, timeout=5,
                                                        config_callback=self.callback)
        self.client.update_configuration(defaults)
        
    def callback(self,config):

        rospy.loginfo("For camera :"+self.topic)
        rospy.loginfo("Config set to auto_white_balance:{auto_white_balance}, white_balance_blue:{white_balance_blue}, white_balance_red:{white_balance_red}, frame_rate:{frame_rate}".format(**config))
    
if __name__ == "__main__":
    rospy.init_node("reconfigure_cameras")

    #the names of variables are picked up from the rqt_reconfigure tool window
    defaults = {"auto_white_balance":False,
                "white_balance_blue":800,
                "white_balance_red":550,
                "frame_rate":2
    }

    left_config = Config("left/camera_nodelet",defaults)
    right_config = Config("right/camera_nodelet",defaults)

    rospy.spin()
