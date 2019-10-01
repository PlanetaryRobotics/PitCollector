#! /usr/bin/env python
import rospy
from labjack_helpers import *
from pan_tilt_helpers import *
import json
import sqlite3
from sqlite3 import Error
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def sql_connection():
    # https://likegeeks.com/python-sqlite3-tutorial/
    # connects to existing database or creates it on first run
    try:
        con = sqlite3.connect('/home/pipedream/PitCollector/src/collector/src/metadata.db')
        return con
    except Error:
        print(Error)

def sql_fetch(con):
    #returns select * all from hardcoded database
    cursorObj = con.cursor()
    cursorObj.execute('SELECT * FROM employees')
    rows = cursorObj.fetchall()
    for row in rows:
        print(row)

def yes_or_no(question):
    reply = str(raw_input(question+' (y/n): ')).lower().strip()
    if reply[0] == 'y':
        return True
    if reply[0] == 'n':
        return False
    else:
        return yes_or_no("please enter y or n")



def main():
    rospy.init_node('collector')
    rate = rospy.Rate(10)

    #establish connection
    con = sql_connection()
    cursorObj = con.cursor()

    #create new table for this position
    cursorObj.execute("CREATE TABLE IF NOT EXISTS employees( \
        id integer PRIMARY KEY AUTOINCREMENT, \
        name text, \
        salary real, \
        department text, \
        position text, \
        hireDate text)")
    con.commit()

    #clean up the table prior to start
    cursorObj.execute("DELETE FROM employees WHERE 1 = 1")
    con.commit()

    #settings = startup_user_interaction()
    
    while not rospy.is_shutdown():

        #writeDIO(0, True)
        #print("Pin 1: "+str(readDIO(1)))

        #loop through the hard-coded position settings
        with open('/home/pipedream/PitCollector/src/collector/src/positions.json') as data_file:
            data = json.load(data_file)

            # movements are hard-cded in logical order
            for length in data['length_inches']:
                print('length: ' + str(length))
                for height in data['height_inches']:
                    print('height: ' + str(height))
                    for pan_tilt in data['pan_tilt']:
                        print('pan_tilt: ' + str(pan_tilt))
                        for shutter_speed in data['shutter_speed']:
                            print('shutter_speed: ' + str(shutter_speed))
                            if yes_or_no('Do you want to take the picture?'):

                                print('Save metadata to sql')
                                cursorObj.execute("INSERT INTO employees VALUES(null, 'John', 700, 'HR', 'Manager', '2017-01-04')")
                                con.commit()

                                print('Save next image being published')
                                print('Success')
                            else:
                                sql_fetch(con)
                                con.close()
                                sys.exit()
        rate.sleep()

def demo_relays():
    rospy.init_node('collector')
    rate = rospy.Rate(10)

    input_pins = range(FIO0, FIO7+1)
    output_pins = range(EIO0, EIO7+1)

    while not rospy.is_shutdown():
        for i in range(8):
            state = readDIO(input_pins[i])
            writeDIO(output_pins[i], state)
        rate.sleep()

def demo_ptu():
    rospy.init_node('collector')
    rate = rospy.Rate(10)

    ptu = PanTilt()
    while not rospy.is_shutdown():
        ptu = PanTilt()
        ptu.setPanTilt(-90, -45)
        ptu.setPanTilt(-60, -45)
        ptu.setPanTilt(-30, -45)
        ptu.setPanTilt(-0, -45)
        ptu.setPanTilt(30, -45)
        ptu.setPanTilt(60, -45)
        ptu.setPanTilt(90, -45)
        ptu.setPanTilt(0, 0)
        rate.sleep()

from sensor_msgs.msg import Imu

import numpy as np
def imuCallback(data):
    quat = data.orientation
    roll,pitch,yaw = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)
    #print(quat) 
    print('roll: %s pitch %s yaw %s' % (roll, pitch, yaw))


def demo_imu():
    rospy.init_node('collector')
    rospy.Subscriber("/imu/data", Imu, imuCallback)
    rospy.spin()


if __name__=="__main__":
    #main()
    #demo_relays()
    demo_ptu()
    #demo_imu()
