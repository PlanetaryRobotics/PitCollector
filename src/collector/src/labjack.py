import rospy
from labjack_node.srv import *

def writeDIO(pin_num, value):
    rospy.wait_for_service('/labjack/write_dio/')
    try:
        write_srv = rospy.ServiceProxy('/labjack/write_dio/', WriteDIO)
        resp = write_srv(pin_num, value)
        return resp
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def readDIO(pin_num):
    rospy.wait_for_service('/labjack/read_dio/')
    try:
        read_srv = rospy.ServiceProxy('/labjack/read_dio/', ReadDIO)
        resp = read_srv(pin_num)
        return resp
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
