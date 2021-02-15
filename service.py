import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
import traceback, os, cv2, time, threading
import pyrealsense2 as rs
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil


class RSImpl(object):
	#Issue: Officially we currently only support 640x480/1024x768 for Depth and 1920x1080/1280x720 for Color.
	def __init__(self, width=640, height=480, fps=30, camera_info=None):
		# Create a pipeline
		self.pipeline = rs.pipeline()

		#Create a config and configure the pipeline to stream
		#  different resolutions of color and depth streams
		self.config = rs.config()
		self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
		self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

		# Start streaming
		self.profile = self.pipeline.start(self.config)

		# Getting the depth sensor's depth scale (see rs-align example for explanation)
		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_scale = self.depth_sensor.get_depth_scale()
		print("Depth Scale is: " , self.depth_scale)

		# Create an align object
		# rs.align allows us to perform alignment of depth frames to others frames
		# The "align_to" is the stream type to which we plan to align depth frames.
		align_to = rs.stream.color
		self.align = rs.align(align_to)

		# Point cloud
		self.pc = rs.pointcloud()
		self.decimate = rs.decimation_filter()
		self.decimate.set_option(rs.option.filter_magnitude, 2 ** 1)


		# self._imaging_consts = RRN.GetConstants('com.robotraconteur.imaging')
		self._image_consts = RRN.GetConstants('com.robotraconteur.image')
		self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
		self._depth_image_type = RRN.GetStructureType('com.robotraconteur.image.DepthImage')
		self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
		# self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')
		# self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
		# self._isoch_info = RRN.GetStructureType('com.robotraconteur.device.isoch.IsochInfo')
		self._capture_lock = threading.Lock()
		self._point_type = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Point")
		self._pointcloud_type = RRN.GetStructureType('com.robotraconteur.pointcloud.PointCloud')
		self._rs_pointcloud_type = RRN.GetStructureType('edu.rpi.robotics.realsense.rs_pointcloud')

		self._streaming = False

		self._sensor_data_util = SensorDataUtil(RRN)
	def _cv_mat_to_image(self, mat):

		is_mono = False
		if (len(mat.shape) == 2 or mat.shape[2] == 1):
			is_mono = True

		image_info = self._image_info_type()
		image_info.width =mat.shape[1]
		image_info.height = mat.shape[0]
		if is_mono:
			image_info.step = mat.shape[1]
			image_info.encoding = self._image_consts["ImageEncoding"]["mono8"]
		else:
			image_info.step = mat.shape[1]*3
			image_info.encoding = self._image_consts["ImageEncoding"]["rgb8"]
		# image_info.data_header = self._sensor_data_util.FillSensorDataHeader(self._camera_info.device_info,self._seqno)
		image = self._image_type()
		image.image_info = image_info
		image.data=mat.reshape(mat.size, order='C')
		return image

	def _mat_to_depthimage(self, mat):
		image = self._depth_image_type()
		image.depth_image=self._cv_mat_to_image(mat)
		image.depth_ticks_per_meter=1./self.depth_scale
		return image

	def _pc_to_RRpc(self,verts,texcoords,w,h):
		rs_pointcloud=self._rs_pointcloud_type()
		RRpc=self._pointcloud_type()
		RRpc.width=w
		RRpc.height=h
		RRpc.points=np.zeros((len(verts)),dtype=self._point_type)

		for i in range(len(verts)):
			RRpc.points[i]['x']=verts[i][0]
			RRpc.points[i]['y']=verts[i][1]
			RRpc.points[i]['z']=verts[i][2]
		rs_pointcloud.pointcloud=RRpc
		rs_pointcloud.texcoords=texcoords.flatten()
		return rs_pointcloud



	def frame_threadfunc(self):
		while(self._streaming):
			with self._capture_lock:
				# Get frameset of color and depth
				frames = self.pipeline.wait_for_frames()
				# frames.get_depth_frame() is a 640x360 depth image

				# Align the depth frame to color frame
				aligned_frames = self.align.process(frames)

				# Get aligned frames
				aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
				color_frame = aligned_frames.get_color_frame()

				# Validate that both frames are valid
				if not aligned_depth_frame or not color_frame:
					continue

				depth_image = np.asanyarray(aligned_depth_frame.get_data(),dtype=np.uint8)
				color_image = np.asanyarray(color_frame.get_data(),dtype=np.uint8)
				#depth data matching with image
				depth_data= np.asarray(aligned_depth_frame.as_frame().get_data())

				self.depth_image_stream.SendPacket(self._mat_to_depthimage(depth_image))
				self.image_stream.SendPacket(self._cv_mat_to_image(color_image))

				###pointcloud part
				depth_frame = self.decimate.process(aligned_depth_frame)

				# Grab new intrinsics (may be changed by decimation)
				depth_intrinsics = rs.video_stream_profile(
					depth_frame.profile).get_intrinsics()
				w, h = depth_intrinsics.width, depth_intrinsics.height

				points = self.pc.calculate(depth_frame)
				self.pc.map_to(color_frame)

				# Pointcloud data to arrays
				v, t = points.get_vertices(), points.get_texture_coordinates()
				verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
				# print(verts)
				texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

				self.pc_stream.SendPacket(self._pc_to_RRpc(verts,texcoords,w,h))


	def StartStreaming(self):
		if (self._streaming):
			raise RR.InvalidOperationException("Already streaming")
		self._streaming=True
		t=threading.Thread(target=self.frame_threadfunc)
		t.start()

	def StopStreaming(self):
		if (not self._streaming):
			raise RR.InvalidOperationException("Not streaming")
		self._streaming=False


def main():
	with RR.ServerNodeSetup("RS_Service", 25415) as node_setup:
		#RR setup
		RRC. RegisterStdRobDefServiceTypes(RRN)
		#register service file and service
		RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.realsense.robdef")


		RS_obj=RSImpl()

		RRN.RegisterService("RS_Service","edu.rpi.robotics.realsense.Realsense",RS_obj)

		input("Press enter to quit")


		RS_obj.pipeline.stop()




if __name__ == "__main__":
	main()