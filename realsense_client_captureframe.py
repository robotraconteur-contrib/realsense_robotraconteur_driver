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

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
	return frame2

def main():
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")
	parser.add_argument("--type",type=str,default='rgb',help="type of image")
	args, _ = parser.parse_known_args()

	cam_dict={'rgb':0,'depth':1}

	url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

	#Startup, connect, and pull out the camera from the objref    
	Multi_Cam_obj=RRN.ConnectService(url)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	cam=Multi_Cam_obj.get_cameras(cam_dict[args.type])
	current_frame=ImageToMat(cam.capture_frame())

	cv2.namedWindow("Image")

	cv2.imshow("Image",current_frame)
	if cv2.waitKey(50)!=-1:
		cv2.destroyAllWindows()

	cam.stop_streaming()



if __name__ == '__main__':
	main()
