import rospy
from ansel.srv import *

def tools_take_3_bracketed_images(camera_topic,file_path,image_count,exposure_time_microseconds):
	rospy.wait_for_service('ansel')
	try:
		ansel_srv = rospy.ServiceProxy('ansel', Ansel)
		req = AnselRequest()
		#service that the camera is publishing to
		req.rostopic = str(camera_topic)
		req.filepath = str(file_path)
		req.image_count = int(image_count)
		req.exposure_time_microseconds = int(exposure_time_microseconds)
		resp = ansel_srv(req)

		rospy.loginfo('camera_tools.py take_3_bracketed_images: Taking and Saving Image. {}'.format(resp))
		return resp
	except rospy.ServiceException, e:
		print("Service call failed: {}".format(e))
		return False


'''
def takeAndSaveImages(camera_topic,file_path,image_count,step_size,base_grey,hdr):
	rospy.wait_for_service('ansel')
	try:
		ansel_srv = rospy.ServiceProxy('ansel', Ansel)
		req = AnselRequest()
		#service that the camera is publishing to
		req.rostopic = str(camera_topic)
		req.filepath = str(file_path)
		#total number of steps and images to take
		req.image_count = int(image_count)
		#step size increases greyscale by step_size%
		req.step_size = int(step_size)
		#base should be 30% greyscale
		req.base_grey = int(base_grey)
		#apply HDR or not
		req.hdr = bool(hdr)
		resp = ansel_srv(req)
		#AnselRequest:
		#string rostopic
		#string filepath
		#uint8 image_count
		#uint8 step_size
		#uint8 base_grey
		#bool hdr
		rospy.loginfo('Taking and Saving Image. {}'.format(resp))
	except rospy.ServiceException, e:
		print("Service call failed: {}".format(e))

	#time.sleep(5)
	return resp
'''