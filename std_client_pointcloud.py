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



def deform_pc(RR_pc):
	return RR_pc.points.view((np.float32, len(RR_pc.points.dtype.names)))

pointcloud=None
graph=None
title=None
#This function is called when a new pipe packet arrives
def new_frame(pipe_ep):
	global pointcloud

	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		sensor_stream=pipe_ep.ReceivePacket()

		try:
			pointcloud=deform_pc(sensor_stream.point_cloud)
		except:
			traceback.print_exc()
		return

def main():
	global pointcloud, graph, title
	url='rr+tcp://localhost:25415?service=PC_Service'
	if (len(sys.argv)>=2):
		url=sys.argv[1]

	#Startup, connect, and pull out the camera from the objref    
	PC_obj=RRN.ConnectService(url)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	p=PC_obj.point_cloud_sensor_data.Connect(-1)

	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p.PacketReceivedEvent+=new_frame


	
	fig = plt.figure()
	ax = p3.Axes3D(fig)
	title = ax.set_title('3D Test')
	time.sleep(2)
	graph = ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2])
	plt.show()

	p.Close()



if __name__ == '__main__':
	main()
