#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *

import cv2, sys, traceback, argparse
import numpy as np


#Function to take the data structure returned from the Webcam service
#and convert it to an OpenCV array
def ImageToMat(image):

	if image.image_info.encoding == image_consts["ImageEncoding"]["bgr888"]:
		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
	elif image.image_info.encoding == image_consts["ImageEncoding"]["depth_u16"]:
		depth_img =image.data.view(dtype=np.uint16).reshape([image.image_info.height, image.image_info.width], order='C')
		frame2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.1), cv2.COLORMAP_JET)
	else:
		assert False, "Unexpected data type"
	return frame2

image_consts=None

def main():
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")
	parser.add_argument("--type",type=str,default='rgb',help="type of image")
	args, _ = parser.parse_known_args()

	cam_dict={'rgb':0,'depth':1}

	url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

	#Startup, connect, and pull out the camera from the objref    
	Multi_Cam_obj=RRN.ConnectService(url)

	global image_consts
	image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	cam=Multi_Cam_obj.get_cameras(cam_dict[args.type])
	current_frame=ImageToMat(cam.capture_frame())

	cv2.namedWindow("Image")

	cv2.imshow("Image",current_frame)
	cv2.waitKey()
	
	cv2.destroyAllWindows()

	



if __name__ == '__main__':
	main()
