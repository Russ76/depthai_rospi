## OAK-D Lite on a Raspberry Pi 5 - *depthai_rospi* package

This is a ROS2 (Jazzy) package designed to work with _OAK-D Lite_ camera on a Raspberry Pi 5.
Based on [depthai_examples](https://github.com/luxonis/depthai-ros/tree/humble/depthai_examples) package.

Luxonis camera delivers data through _pipelines_, and when running at high frame rate (e.g. 30 FPS) - it overwhelmes ROS network (with data rate over 700 Mbits/s).

There is a known issue - _OAK-D Lite_ doesn't support changing FPS via `i_fps` parameter or in code:
https://github.com/luxonis/depthai/issues/624

In my experiments with OAK-D Lite, supplying correct `.yaml` file as [ROS parameter](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md#configuration-files) worked fine, lowering FPS as intended.

If `i_fps` doesn't have effect, the code in this package tries to read the pipeline as-is, but publishes only _some_ of the data.
This makes rates of _Image_ and _PointCloud2_ topics acceptable for WiFi connected robots.
It also runs _Publishers_ in a separate thread, and restarts it if device pipelines freeze.

**Note:** review this [guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md) first.  

We assume that your _OAK-D Lite_ camera is connected to Raspberry Pi, while you view data on your Workstation (Desktop PC).
Both machines should run Ubuntu 24.04 - and Raspberry Pi can be "headless" (Server version of the OS).

## Build

First, install standard Luxonis [ROS packages](https://github.com/luxonis/depthai-ros/tree/jazzy) on Raspberry Pi and on the Desktop machine :
```
sudo apt install ros-${ROS_DISTRO}-depthai-ros
```
Now you can clone and build this customized package on Raspberry Pi:
```
mkdir -p ~/depthai_ws/src
cd ~/depthai_ws/src
git clone https://github.com/slgrobotics/depthai_rospi.git
cd ~/depthai_ws
colcon build
```

## Usage

#### Stereo Publisher:

Here is how to launch the _stereo publisher node_ and its conversion helpers:
```
cd ~/depthai_ws
source install/setup.bash
ros2 launch depthai_rospi stereo_pi.launch.py
```
To see your data in RViz2 on the Desktop machine, follow this [guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md#on-the-desktop-machine)

By default, the `fpsDivider` parameter is set to 10, which amounts to about 3 FPS on publishing all data (left and right Mono images and Depth).
All data is published raw (uncompressed).
The load on the WiFi link is around 120 Mbits/s for _PointCloud2_ data and about 8 Mbits/s for _Image_ data (run `nload wlan0` on RPi).

#### Spatial with 3D Detection:
Here is how to launch the *spatial_bb*, similar to an [example](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md#spatial-examples) from *depthai_filters*:
```
ros2 launch depthai_rospi spatial_bb.launch.py
```
To see */overlay* and */oak/points* topics in RVis2, use settings here:
```
~/depthai_ws/src/depthai_rospi/rviz/default.rviz
```

## Useful Links

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md

