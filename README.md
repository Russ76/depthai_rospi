## *depthai_rospi* package

This is a ROS2 (Jazzy) package designed to work with _OAK-D Lite_ camera on a Raspberry Pi 5.
Based on [depthai_examples](https://github.com/luxonis/depthai-ros/tree/humble/depthai_examples) package.

Luxonis cameras deliver data through _pipelines_, and when running at high FPS (e.g. 30 FPS) - it overwhelmes the ROS network.

There is a known issue - OAK-D Lite doesn't support changing FPS via `i_fps` parameter or in code:
https://github.com/luxonis/depthai/issues/624

The code in this package tries to read the pipeline as-is, but publishes only some of the data,
thus making rates of _Image_ and _PointCloud2_ topics acceptable for WiFi connected robots.

## Build

You should first install standard Luxonis ROS packages:
```
sudo apt install ros-${ROS_DISTRO}-depthai-ros
```
Now you can clone and build this customized package:
```
mkdir -p ~/depthai_ws/src
cd ~/depthai_ws/src
git clone https://github.com/slgrobotics/depthai_rospi.git
cd ~/depthai_ws
colcon build
```

## Usage

```
cd ~/depthai_ws
source install/setup.bash
ros2 launch depthai_rospi stereo_pi.launch.py
```
To see your data in RViz2, follow the guide below:

## Useful Links

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md

