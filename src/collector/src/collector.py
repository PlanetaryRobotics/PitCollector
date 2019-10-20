#! /usr/bin/env python
import rospy
import numpy as np

from rig import Rig
from camera import Camera
from Tkinter import *
from PIL import ImageTk,Image
import tkFileDialog
import Tkinter as tk
import ScrolledText as tkst
import time

import json

class GUI:
    def __init__(self):
        self.rig = Rig()
        #the switch can be left on from previoius hanging run. need to kill it ASAP
        self.rig.kill_all_motors()

        self.root = Tk()
        self.camera = Camera()
        self.root.title("PitCollector")

        # Image Display Canvas
        self.canvas = Canvas(self.root, width=500, height=300)
        self.canvas.grid(row=0, column=0, rowspan=5, columnspan=1, padx=50, pady=50)

        # Roll Label
        self.roll_label = Label(self.root)
        self.roll_label.grid(row=5, column=0, pady=30)

        # Pitch Label
        self.pitch_label = Label(self.root)
        self.pitch_label.grid(row=6, column=0, pady=30)

        # Name Label
        self.name_label = Label(self.root, text="Location Name \"location_1\":")
        self.name_label.grid(row=0, column=1, pady=50)
        # Name Entry
        self.name_entry = Entry(self.root)
        self.name_entry.grid(row=0, column=2, padx=30)

        # Comments Label
        self.comments_label = Label(self.root, text="Comments:")
        self.comments_label.grid(row=2, column=1)
        self.comments_text = tkst.ScrolledText(self.root, width=50, height=8)
        self.comments_text.grid(row=2, column=2, columnspan=3, padx=30)

        # Home Button
        self.home_btn = Button(self.root, text="Home")
        self.home_btn.grid(row=3, column=1)
        # Checkout Button
        self.checkout_btn = Button(self.root, text="Checkout")
        self.checkout_btn.grid(row=3, column=2)
        # Start Button
        self.start_btn = Button(self.root, text="Start")
        self.start_btn.grid(row=3, column=3)
        # Abort Button
        self.abort_btn = Button(self.root, text="Abort")
        self.abort_btn.grid(row=3, column=4)

        self.rig.kill_all_motors()
        self.root.after(100, self.update)
        img = ImageTk.PhotoImage(Image.open("/home/pipedream/Downloads/test.jpg"))
        self.canvas.create_image(10, 10, anchor=NW, image=img)

    def update(self):
        if not rospy.is_shutdown():
            self.update_roll_pitch_labels()

            self.rig.update()
            self.root.after(100, self.update)
        else:
            self.root.destroy()

    def update_roll_pitch_labels(self):
        roll, pitch = self.rig.get_roll_pitch()
        self.roll_label.config(text="Roll (deg): {0:.2f}".format(roll))
        self.pitch_label.config(text="Pitch (deg): {0:.2f}".format(pitch))

    def mainloop(self):
        #update the GUI image with the latest image from the camera
        self.rig.kill_all_motors()
        self.root.after(100, self.update)
        img = ImageTk.PhotoImage(Image.open("/home/pipedream/Downloads/test.jpg"))
        self.canvas.create_image(10, 10, anchor=NW, image=img)

        #there seems to be a lag inputting pins from the Labjack. Adding a short sleep so we don't call 
        #any movement functions that depend on it

        #print('goto F2')
        #print(self.rig.x_axis_go_to('F2'))

        #print('set y0 axis')
        #print(self.rig.y0_axis_set_up())

        #why does this output empty set???
        print(self.rig.return_true_pin_names(self.rig.read_all_input_pins()))

        #print('collector: check for positive pins y1')
        #print(self.rig.check_for_positive_pins(['F6','F7']))

        #print('collector: y1 go to F6')
        #print(self.rig.y1_axis_go_to('F6'))
        #print('collector: y1 goto F7')
        #print(self.rig.y1_axis_go_to('F7'))
        #print('collector: y1 goto F6')
        #print(self.rig.y1_axis_go_to('F6'))

        #print('collector: y0 go to F4')
        #print(self.rig.y0_axis_go_to('F4'))
        #print('collector: y0 goto F5')
        #print(self.rig.y0_axis_go_to('F5'))

        #print('collector: x axis goto F0')
        #print(self.rig.x_axis_go_to('F0'))
        #print('collector: goto F1')
        #print(self.rig.x_axis_go_to('F1'))

        #print('goto F0')
        #print(self.rig.x_axis_go_to('F0'))
        #print('goto F1')
        #print(self.rig.x_axis_go_to('F1'))

        # Test run the ansel module
        #rospy.loginfo('saving images in collector node')
        #resp = self.camera.take_and_save_images('/camera/image_color','../PitCollector/data',3,20,30,True)
        #rospy.loginfo(resp)
        #camera_topic,file_path,image_count,step_size,base_grey,hdr]

        #import os
        #cwd = os.getcwd()
        #print(cwd)
        self.rig.go_home_no_safeguard()
        self.rig.run_full_sequence('/home/pipedream/PitCollector/json_sequences/sequence_of_positions.json')
        self.root.mainloop()

def kill_ros():
    import subprocess
    subprocess.call(["rosnode", "kill", "--all"])
    subprocess.call(["killall", "-9", "rosmaster"])
    subprocess.call(["killall", "-9", "roscore"])

if __name__ == "__main__":
    rospy.init_node('collector')
    gui = GUI()
    gui.mainloop()



    if not rospy.is_shutdown():
        #kill all motors in event of abort
        gui.rig.kill_all_motors()
        kill_ros()
