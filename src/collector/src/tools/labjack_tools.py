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

pinMap = {\
     'F0':FIO0, 'F1':FIO1, 'F2':FIO2, 'F3':FIO3, 'F4':FIO4, 'F5':FIO5, \
     'F6':FIO6, 'F7':FIO7, 'E0':EIO0, 'E1':EIO1, 'E2':EIO2, 'E3':EIO3, \
     'E4':EIO4, 'E5':EIO5, 'E6':EIO6, 'E7':EIO7 }

def writeDIO(pin_name, value):
    rospy.wait_for_service('/labjack/write_dio/')
    if pinMap[pin_name] in [EIO1,EIO3,EIO5]:
        try:
            write_srv = rospy.ServiceProxy('/labjack/write_dio/', WriteDIO)
            resp = write_srv(pinMap[pin_name], value)
            #print('writeDIO success')
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
    else:
        print('Invalid pin for writing to in writeDIO ', pin_name )

#separate motor writes from direction writes
def writeMotorsDIO(pin_name, value):
    rospy.wait_for_service('/labjack/write_dio/')
    if pinMap[pin_name] in [EIO0,EIO2,EIO4]:
        try:
            write_srv = rospy.ServiceProxy('/labjack/write_dio/', WriteDIO)
            resp = write_srv(pinMap[pin_name], value)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
    else:
        print('Invalid pin for writing to in writeMotorsDIO ', pin_name )

def readDIO(pin_name):
    rospy.wait_for_service('/labjack/read_dio/')
    #these are the only valid pins that should be read/write
    if pinMap[pin_name] in [FIO0,FIO1,FIO2,FIO3,FIO4,FIO5,FIO6,FIO7]:
        try:
            read_srv = rospy.ServiceProxy('/labjack/read_dio/', ReadDIO)
            resp = read_srv(pinMap[pin_name])
            return resp.value
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
    else: 
        print('Invalid pin entered for reading in ReadDIO ', pin_num)