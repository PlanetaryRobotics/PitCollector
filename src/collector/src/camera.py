import rospy
from ansel.srv import *

class Camera:
    def __init__(self):
        self.camera_topic = ''
        self.file_path = ''
        self.image_count = 1
        self.exposure_time_microseconds = 50000
        self.bracketing_factor =1

    def take_n_bracketed_images(self, camera_topic,data_filepath,image_count, \
        exposure_time_microseconds,bracketing_factor, \
        current_position,pan,tilt):

        rospy.wait_for_service('ansel')
        try:
            ansel_srv = rospy.ServiceProxy('ansel', Ansel)
            req = AnselRequest()
            #service that the camera is publishing to
            req.rostopic = str(camera_topic)
            req.data_filepath = str(data_filepath)
            req.image_count = int(image_count)
            req.exposure_time_microseconds = int(exposure_time_microseconds)
            req.bracketing_factor = float(bracketing_factor)
            req.position = current_position
            #convert the pan/tilt from negative to a file friendly name
            req.pan = str(  int(round(pan,0)) )
            req.tilt = str( int(round(tilt,0)) )
            resp = ansel_srv(req)
            rospy.loginfo('camera_tools.py take_n_bracketed_images: Taking and Saving Image. {}'.format(resp))
            return resp
        except rospy.ServiceException, e:
            print("Service call failed: {}".format(e))
            return False

    def update(self):
        pass
