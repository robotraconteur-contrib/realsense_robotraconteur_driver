import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
import traceback, os, cv2, time, threading, argparse, yaml
import pyrealsense2 as rs
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
import drekar_launch_process


class RGB_Cam(object):
	def __init__(self,RS_obj):
		self._streaming = False
		self._capture_lock = threading.Lock()
		self.RS_obj=RS_obj

	def start_streaming(self):
		self._streaming=True
		self.RS_obj.update_streaming()


	def stop_streaming(self):
		self._streaming=False
		self.RS_obj.update_streaming()
	def capture_frame(self):
		return self.RS_obj.capture_rgb_frame()

class Depth_Cam(object):
	def __init__(self,RS_obj):
		self._streaming = False
		self._capture_lock = threading.Lock()
		self.RS_obj=RS_obj
	def start_streaming(self):
		self._streaming=True
		self.RS_obj.update_streaming()


	def stop_streaming(self):
		self._streaming=False
		self.RS_obj.update_streaming()

	def capture_frame(self):
		return self.RS_obj.capture_depth_frame()

class Multi_Cam(object):
	def __init__(self,RS_obj):
		self._streaming = False
		self._capture_lock = threading.Lock()
		self.RGB_Cam_obj=RGB_Cam(RS_obj)
		self.Depth_Cam_obj=Depth_Cam(RS_obj)
		self.cameras=[self.RGB_Cam_obj,self.Depth_Cam_obj]
		self.camera_names=['RGB','Depth']
		self._image_consts = RRN.GetConstants('com.robotraconteur.image')
		self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
		self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
		
	def get_cameras(self,ind):
		int_ind=int(ind)
		return self.cameras[int_ind], "com.robotraconteur.imaging.Camera"
	def _cv_mat_to_image(self, mat, is_depth=False):

		is_mono = False
		if (len(mat.shape) == 2 or mat.shape[2] == 1):
			is_mono = True

		image_info = self._image_info_type()
		image_info.width =mat.shape[1]
		image_info.height = mat.shape[0]
		if is_depth:
			image_info.step = mat.shape[1]*2
			image_info.encoding = self._image_consts["ImageEncoding"]["depth_u16"]
		elif is_mono:
			image_info.step = mat.shape[1]
			image_info.encoding = self._image_consts["ImageEncoding"]["mono8"]
		else:
			image_info.step = mat.shape[1]*3
			image_info.encoding = self._image_consts["ImageEncoding"]["bgr888"]

		image = self._image_type()
		image.image_info = image_info
		image.data=mat.reshape(mat.size, order='C').view(dtype=np.uint8)
		return image

class PC_Sensor(object):
	def __init__(self,RS_obj):
		self._active=False
		self._point_type = RRN.GetNamedArrayDType("com.robotraconteur.geometryf.Point")
		self._pointcloud_type = RRN.GetStructureType('com.robotraconteur.pointcloud.PointCloudf')
		self._pointcloudsensordata_type=RRN.GetStructureType('com.robotraconteur.pointcloud.sensor.PointCloudSensorData')
		self.RS_obj=RS_obj

	def _pc_to_RRpc(self,verts,texcoords,w,h):
		RRpc=self._pointcloud_type()
		#RRpc.width=w
		#RRpc.height=h
		RRpc.points=np.zeros((len(verts)),dtype=self._point_type)
		RRpc.points=RRN.ArrayToNamedArray(verts,named_array_dt=self._point_type)
		return RRpc
	def _RRpc_to_PCSD(self,RRpc):
		PCSD=self._pointcloudsensordata_type()
		PCSD.point_cloud=RRpc
		return PCSD

	def capture_point_cloud(self):
		return self.RS_obj.capture_point_cloud()

	@property
	def active(self):
		return self._active

	@active.setter
	def active(self,v):
		self._active = v
		self.RS_obj.update_streaming()

class RSImpl(object):
	#Resolution: 1920x1080, 1280x720 or 640x480 for RGB, 1280x720 or 640x480 for depth
	#FPS: 60/30/15/6
	def __init__(self, rgb_res=(1920,1080), depth_res=(1280,720), fps=15):
		#initialize RR obj
		self.Multi_Cam_obj=Multi_Cam(self)
		self.PC_Sensor=PC_Sensor(self)

		# Create a pipeline
		self.pipeline = rs.pipeline()

		#Create a config and configure the pipeline to stream
		#  different resolutions of color and depth streams
		self.config = rs.config()
		self.config.enable_stream(rs.stream.depth, depth_res[0], depth_res[1], rs.format.z16, fps)
		self.config.enable_stream(rs.stream.color, rgb_res[0], rgb_res[1], rs.format.bgr8, fps)

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


		
		self._capture_lock = threading.Lock()
		

		self._streaming = False

	def capture_rgb_frame(self):
		with self._capture_lock:
			# Get frameset of color and depth
			frames = self.pipeline.wait_for_frames()

			# Align the depth frame to color frame
			aligned_frames = self.align.process(frames)

			# Get color frame
			color_frame = aligned_frames.get_color_frame()
			color_image = np.asanyarray(color_frame.get_data(),dtype=np.uint8)
		return self.Multi_Cam_obj._cv_mat_to_image(color_image)

	def capture_depth_frame(self):
		with self._capture_lock:
			# Get frameset of color and depth
			frames = self.pipeline.wait_for_frames()

			# Align the depth frame to color frame
			aligned_frames = self.align.process(frames)

			# Get color frame
			aligned_depth_frame = aligned_frames.get_depth_frame()
			depth_image = np.array(aligned_depth_frame.get_data(),dtype=np.uint16)
			
		return self.Multi_Cam_obj._cv_mat_to_image(depth_image,True)

	def capture_point_cloud(self):
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
				raise ('something wrong here')

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
			texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

			return self.PC_Sensor._pc_to_RRpc(verts,texcoords,w,h)


	def frame_threadfunc(self):
		while(self._streaming):
			with self._capture_lock:
				try:
					# Get frameset of color and depth
					frames = self.pipeline.wait_for_frames()
					# frames.get_depth_frame() is a 640x360 depth image

					# Align the depth frame to color frame
					aligned_frames = self.align.process(frames)

					# Get aligned frames
					aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
					color_frame = aligned_frames.get_color_frame()
					color_image = np.asanyarray(color_frame.get_data(),dtype=np.uint8)

					# Validate that both frames are valid
					if not aligned_depth_frame or not color_frame:
						continue

					depth_image = np.array(aligned_depth_frame.get_data(),dtype=np.uint16)
					#depth data matching with image
					#depth_data= np.asarray(aligned_depth_frame.as_frame().get_data())

					#if self.Multi_Cam_obj.cameras[0]._streaming:
					rgb_frame_stream = None
					try:			
						rgb_frame_stream = self.Multi_Cam_obj.cameras[0].frame_stream
					except AttributeError:
						pass
					if rgb_frame_stream:
						rgb_frame_stream.SendPacket(self.Multi_Cam_obj._cv_mat_to_image(color_image))
					#if self.Multi_Cam_obj.cameras[1]._streaming:
					depth_frame_stream = None
					try:
						depth_frame_stream = self.Multi_Cam_obj.cameras[1].frame_stream
					except AttributeError:
						pass
					if depth_frame_stream:
						depth_frame_stream.SendPacket(self.Multi_Cam_obj._cv_mat_to_image(depth_image,True))
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
					texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

					PCSD=self.PC_Sensor._RRpc_to_PCSD(self.PC_Sensor._pc_to_RRpc(verts,texcoords,w,h))
					if self.PC_Sensor._active:
						self.PC_Sensor.point_cloud_sensor_data.SendPacket(PCSD)
				except:
					traceback.print_exc()

	def update_streaming(self):
		
		need_stream = False
		if self.Multi_Cam_obj.cameras[0]._streaming:
			need_stream = True
		if self.Multi_Cam_obj.cameras[1]._streaming:
			need_stream = True
		if self.PC_Sensor._active:
			need_stream = True

		if need_stream: 
			if not self._streaming:
				self._streaming = True
				t=threading.Thread(target=self.frame_threadfunc)
				t.start()
		else:
			self._streaming = False




def main():
	with RR.ServerNodeSetup("RS_Node", 25415) as node_setup:
		parser = argparse.ArgumentParser(description="Realsense Driver for Robot Raconteur")
		parser.add_argument("--realsense-info-file", type=argparse.FileType('r'),required=True,help="Realsense info file")
		parser.add_argument("--rgb-resolution", nargs='+',type=int,default=[640, 480],help="RGB camera resolution, up to 1920x1080")
		parser.add_argument("--depth-resolution", nargs='+',type=int,default=[640, 480],help="Depth camera resolution, up to 1280x720")
		parser.add_argument("--fps",type=int,default=60,help="fps, up to 60, subject to resolution")
		args, _ = parser.parse_known_args()

		#Register Service types
		RRC. RegisterStdRobDefServiceTypes(RRN)
		#create object
		RS_obj=RSImpl(rgb_res=tuple(args.rgb_resolution),depth_res=tuple(args.depth_resolution),fps=args.fps)

		with args.realsense_info_file:
			realsense_info_yml = yaml.safe_load(args.realsense_info_file)
		info_loader = InfoFileLoader(RRN)
		multicam_info, multicam_ident_fd = info_loader.LoadInfoFileFromString(yaml.safe_dump(realsense_info_yml["multicamera"]), "com.robotraconteur.imaging.camerainfo.MultiCameraInfo", "device")
		point_cloud_info, multicam_ident_fd = info_loader.LoadInfoFileFromString(yaml.safe_dump(realsense_info_yml["point_cloud"]), "com.robotraconteur.pointcloud.sensor.PointCloudSensorInfo", "device")
		attributes_util = AttributesUtil(RRN)
		multicam_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(multicam_info.device_info)
		service_ctx1 = RRN.RegisterService("Multi_Cam_Service","com.robotraconteur.imaging.MultiCamera",RS_obj.Multi_Cam_obj)
		service_ctx1.SetServiceAttributes(multicam_attributes)
		point_cloud_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(point_cloud_info.device_info)
		service_ctx2 = RRN.RegisterService("PC_Service","com.robotraconteur.pointcloud.sensor.PointCloudSensor",RS_obj.PC_Sensor)
		service_ctx2.SetServiceAttributes(point_cloud_attributes)
		
		# RS_obj.Multi_Cam_obj.cameras[0]._streaming=True
		# RS_obj.Multi_Cam_obj.cameras[1]._streaming=True
		# RS_obj.PC_Sensor._active=True
		# RS_obj.update_streaming()

		print("Press Ctrl-C to quit")

		drekar_launch_process.wait_exit()

		# RS_obj.Multi_Cam_obj.cameras[0]._streaming=False
		# RS_obj.Multi_Cam_obj.cameras[1]._streaming=False
		# RS_obj.PC_Sensor._active=False
		# RS_obj.update_streaming()

		RS_obj.pipeline.stop()




if __name__ == "__main__":
	main()