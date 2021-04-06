# Realsense Depth Camera Robot Raconteur Driver

**WARNING: THIS DRIVER IS A WORK IN PROGRESS.**

## Prerequisites:
* python3: python3.7 for Windows, python 3.8 for Ubuntu (python3 refers 3.7 or 3.8 according to your system settings)
* Robot Raconteur (Windows: `python3 -m pip install robotraconteur`,Ubuntu:
 ```
sudo add-apt-repository ppa:robotraconteur/ppa
sudo apt-get update
sudo apt-get install python3-robotraconteur
```

* Robot Raconteur Companion (`python3 -m pip install RobotRaconteurCompanion`)
* OpenCV (`python3 -m pip install python-opencv`)
* Matplotlib (`python3 -m pip install matplotlib`)
* pyrealsense2 (`python3 -m pip install pyrealsense2`)

## Standard Service Types:
* `Multi_Cam_Service`:
RGB Image and Depth Image
([com.robotraconteur.imaging.MultiCamera](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/942d4f094eb5f686ce86188547a6b470192e045b/group1/com.robotraconteur.imaging.robdef#L90))
* `PC_Service`
Point Cloud Sensor
([com.robotraconteur.pointcloud.sensor.PointCloudSensor](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/group1/com.robotraconteur.pointcloud.sensor.robdef#L47))
## Start RR Service
`python3 realsense_service.py`


## RR Client Example
* RGB Image View
`python realsense_client_image.py --type=rgb`
* Depth Image View
`python realsense_client_image.py --type=depth`
* Pointcloud View
`python realsense_client_pointcloud.py`
![Pointcloud_view](pointcloud_view.png)
