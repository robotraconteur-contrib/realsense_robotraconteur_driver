#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *

import time, math
import numpy as np
import sys, traceback
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


def main():
	global pointcloud, graph, title
	url='rr+tcp://localhost:25415?service=PC_Service'
	
	#Startup, connect, and pull out the camera from the objref    
	PC_obj=RRN.ConnectService(url)
	pointcloud_rr = PC_obj.capture_point_cloud()
	pointcloud = RRN.NamedArrayToArray(pointcloud_rr.points)
	
	fig = plt.figure()
	ax = fig.add_subplot(projection="3d")
	title = ax.set_title('3D Test')
	graph = ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2])
	plt.show()

if __name__ == '__main__':
	main()
