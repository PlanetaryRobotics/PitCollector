import rospy
from labjack_node.srv import *

# Digital I/O Pin Numbers
DIO0 = 0
DIO1 = 1
DIO2 = 2
DIO3 = 3
DIO4 = 4
DIO5 = 5
DIO6 = 6
DIO7 = 7
DIO8 = 8
DIO9 = 9
DIO10 = 10
DIO11 = 11
DIO12 = 12
DIO13 = 13
DIO14 = 14
DIO15 = 15
DIO16 = 16
DIO17 = 17
DIO18 = 18
DIO19 = 19
DIO20 = 20
DIO21 = 21
DIO22 = 22

# Digital I/O Pin Aliases
FIO0 = DIO0
FIO1 = DIO1
FIO2 = DIO2
FIO3 = DIO3
FIO4 = DIO4
FIO5 = DIO5
FIO6 = DIO6
FIO7 = DIO7

EIO0 = DIO8
EIO1 = DIO9
EIO2 = DIO10
EIO3 = DIO11
EIO4 = DIO12
EIO5 = DIO13
EIO6 = DIO14
EIO7 = DIO15

def writeDIO(pin_num, value):
    rospy.wait_for_service('/labjack/write_dio/')
    try:
        write_srv = rospy.ServiceProxy('/labjack/write_dio/', WriteDIO)
        resp = write_srv(pin_num, value)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def readDIO(pin_num):
    rospy.wait_for_service('/labjack/read_dio/')
    try:
        read_srv = rospy.ServiceProxy('/labjack/read_dio/', ReadDIO)
        resp = read_srv(pin_num)
        return resp.value
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
