# Realsense Depth Camera Robot Raconteur Driver

## Prerequisites:
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download)
* python3.7 (for RR service only)
* pyrealsense2 (`python3.7 -m pip install pyrealsense2')


## Start RR Service
`python3.7 realsense_service.py`


## RR Client Example
* RGB Image View
`python realsense_client_image.py --type=rgb`
* Depth Image View
`python realsense_client_image.py --type=depth`
* Pointcloud View
`python realsense_client_pointcloud.py`
