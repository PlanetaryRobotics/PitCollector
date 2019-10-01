import rospy
import std_msgs.msg
from pan_tilt_driver.srv import *
import time

def setPanTilt(pan_deg, tilt_deg):
    rospy.wait_for_service('ptu/goto')
    try:
        ptu_srv = rospy.ServiceProxy('ptu/goto', ptugoto)
        req = ptugotoRequest()
        req.pan = int(pan_deg)
        req.tilt = int(tilt_deg)
        resp = ptu_srv(req)
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

    time.sleep(5)
