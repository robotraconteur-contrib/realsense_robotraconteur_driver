# Realsense Depth Camera Robot Raconteur Driver

## Prerequisites:
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download)
* python3.7 (for RR service only)
* pyrealsense2 (`python3.7 -m pip install pyrealsense2`)

## Standard Service Types:
* `Multi_Cam_Service`:
RGB Image and Depth Image
([com.robotraconteur.imaging.MultiCamera](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/942d4f094eb5f686ce86188547a6b470192e045b/group1/com.robotraconteur.imaging.robdef#L90))
* `PC_Service`
Point Cloud Sensor
([com.robotraconteur.pointcloud.sensor.PointCloudSensor](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/group1/com.robotraconteur.pointcloud.sensor.robdef#L47))
## Start RR Service
`python3.7 realsense_service.py`


## RR Client Example
* RGB Image View
`python realsense_client_image.py --type=rgb`
* Depth Image View
`python realsense_client_image.py --type=depth`
* Pointcloud View
`python realsense_client_pointcloud.py`
