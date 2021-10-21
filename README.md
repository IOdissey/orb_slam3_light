### orb_slam3_light

This is fork of the [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) without Pangolin for ROS usage.

### Dependencies

* [Eigen3](https://gitlab.com/libeigen/eigen/-/tags)
* [OpenCV](https://opencv.org) (at least 3.0)
* [ROS](http://wiki.ros.org/ROS/Installation) (for ros example)

### Build

Build all (DBoW2, g2o, ORB_SLAM3, ros_orb_slam3):
```
mkdir build
cd build
cmake <path to orb_slam3_light>
make
```

### ros_orb_slam3

Run:

```
cd orb_slam3_light/ros/ros_orb_slam3/bin
./ros_orb_slam3 <path to config file>
```

The example config file is located:
```
orb_slam3_light/ros/ros_orb_slam3/cfg.yaml
```
