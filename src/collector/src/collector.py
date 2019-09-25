#! /usr/bin/env python
import rospy

def main():
    rospy.init_node('collector')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Do things here.
        print("Hello, PitCollector!")
        rate.sleep()


if __name__=="__main__":
    main()
