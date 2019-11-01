#! /usr/bin/env python
import rospy
import numpy as np

from rig import Rig

from Tkinter import *
from PIL import ImageTk,Image
import tkFileDialog
import Tkinter as tk
import ScrolledText as tkst
import time
import json

#includes for grabbing image from topic stream and outputting it to GUI
#https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import cv2
import dynamic_reconfigure.client
import rospy

from std_msgs.msg import Float32




class GUI:
    def __init__(self):
        self.rig = Rig()
        #the switch can be left on from previoius hanging run. need to kill it ASAP
        self.rig.kill_all_motors()

        self.root = Tk()
        #self.camera = Camera()
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

        # Camera Labels
        self.exposure_label = Label(self.root)
        self.exposure_label.grid(row=7, column=0, pady=30)

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

        self.img = ImageTk.PhotoImage(Image.open("/home/pipedream/Downloads/test.jpg"))
        self.canvas.create_image(10, 10, anchor=NW, image=self.img)

        #https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/
        rospy.Subscriber("/camera/image_raw", ImageMSG, self.image_callback)
        self.bridge = CvBridge()
        self.image_stream_count = 0

        self.current_exposure = -1
        rospy.Subscriber("/camera/exposure_time", Float32, self.exposure_callback)

        self.rig.kill_all_motors()
        self.root.after(100, self.update)

        #to resolve conflict with GUI and image callback, lets create a local copy
        #https://stackoverflow.com/questions/16235955/create-a-multichannel-zeros-mat-in-python-with-cv2/16236373
        self.cv_mat = np.zeros((3248, 4096, 3), dtype = "uint8")

    def exposure_callback(self,msg):
        self.current_exposure = msg.data

    def update_camera_labels(self):
        self.exposure_label.config(text="Exposure (us): {}".format(self.current_exposure))

    def update_GUI_image(self):
        #update the GUI with the image from the stream
        self.img = ImageTk.PhotoImage(Image.open('../PitCollector/GUI.jpeg'))
        if self.img is not None:
            pass
            #print('collector: TK image saved.')
        self.canvas.create_image(20, 20, anchor=NW, image=self.img)
        self.canvas.image = self.img

    def image_callback(self, msg):
        #print('collector: in gui image_callback')
        try:
            # grab every nth image
            if self.image_stream_count % 3 == 0 :
                #print("collector: streaming received image from image_raw")
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                #shrink the camera ouput for the GUI
                cv2_img_small = cv2.resize(cv2_img,(500,500))
                if cv2_img_small is not None:
                    pass
                    #print('collector: ROS image converted for GUI')
                if cv2.imwrite('../PitCollector/GUI.jpeg', cv2_img_small):
                    pass
                    #print('collector: GUI image saved')

                #img = cv2.imread('/tmp/1.jpg')
                #print cv2_img.shape, cv2_img.dtype
        except CvBridgeError, e:
            print(e)
        else:
            pass
        self.image_stream_count += 1

    def update(self):
        if not rospy.is_shutdown():
            #self.update_roll_pitch_labels()
            #self.update_camera_labels()
            #self.update_GUI_image()
            #self.rig.update()
            #self.root.after(100, self.update)
        else:
            self.root.destroy()

    def update_roll_pitch_labels(self):
        roll, pitch = self.rig.get_roll_pitch()
        self.roll_label.config(text="Roll (deg): {0:.2f}".format(roll))
        self.pitch_label.config(text="Pitch (deg): {0:.2f}".format(pitch))

######################################################3
    def main(self):
        #how to run Tkinter without main loop
        #https://gordonlesti.com/use-tkinter-without-mainloop/
        self.rig.kill_all_motors()
        #self.root.after(100, self.update)
        #self.root.mainloop()
        while not rospy.is_shutdown():
            #self.rig.go_home_no_safeguard()
            #self.update()
            #self.root.update()
            print('collector: in main loop')
            gui.rig.run_full_sequence('/home/pipedream/PitCollector/json_sequences/test_run.json',5,100000)
            print('collector: main loop complete')
        #print('collector: running camera sequence')
        #pass data folder, image_count, baseline exposure_time_microseconds
######################################################

def kill_ros():
    import subprocess
    subprocess.call(["rosnode", "kill", "--all"])
    subprocess.call(["killall", "-9", "rosmaster"])
    subprocess.call(["killall", "-9", "roscore"])

if __name__ == "__main__":
    rospy.init_node('collector')
    gui = GUI()
    gui.main()
    if not rospy.is_shutdown():
        #kill all motors in event of abort
        gui.rig.kill_all_motors()
        kill_ros()

