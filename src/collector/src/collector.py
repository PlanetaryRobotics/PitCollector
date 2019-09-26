#! /usr/bin/env python
import rospy
from labjack import writeDIO, readDIO

def main():
    rospy.init_node('collector')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Do things here.
        print("Hello, PitCollector!")
        writeDIO(0, True)
        print("Pin 1: "+str(readDIO(1)))
        rate.sleep()


if __name__=="__main__":
    main()
