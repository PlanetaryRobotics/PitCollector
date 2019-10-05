import rospy
from ansel.srv import *

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
		resp = anser_srv(req)
		#AnselRequest:
		#string rostopic
		#string filepath
		#uint8 image_count
		#uint8 step_size
		#uint8 base_grey
		#bool hdr
		ROS_INFO('Taking and Saving Image. {}'.format(resp))
	except (rospy.ServiceException, rospy.ROSException), e:
		print("Service call failed: {}".format(e))

	time.sleep(5)
	return resp
