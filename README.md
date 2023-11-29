# Green Gaurdian AMR
## Introduction
This is a project for an autonomous mobile robot equipped with a manipulator to be used for agricultural purposes. The robot is designed to be able to navigate autonomously in a greenhouse, detect and classify plants, and perform tasks such as watering and pruning.

## Setup
Import ThirdParty packages
```bash
cd src/ThirdParty
vcs import < ../ros2.repos
```

## I2C Communication
```bash
chmod 666 /dev/i2c-1
```
## 6DOF Arm Arduino Sketch
limits: 10-170

## Visualize
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
docker run --rm -p "8080:8080" ghcr.io/foxglove/studio:latest
```

sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen