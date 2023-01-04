# Touch

ROS packages for interfacing with 3D Systems Touch haptic devices.


## The Touch base coordinate system

- Right is +X.
- Up is +Y.
- Toward user is +Z.


## Driver / SDK download

https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34


## Ubuntu Jammy Docker container

We have experienced issues with the Touch devices on Ubuntu Jammy (22.04). The 3D Systems drivers are not officially supported on Jammy. Thus we provide the means to run `touch_control` nodes in a Docker container based on Ubuntu Focal (20.04) with ROS2 Foxy.

The `docker/fastrtps_profile.xml` forces UDP communication (instead of shared memory) from inside the container.

The 3D Systems config directory is shared with the host and so the device may be configured by running `Touch_Setup` on either the host or container.

Build Docker image:

```
docker build -t touch-focal -f docker/Dockerfile .
```

Run `Touch_Setup` configuration program (a Qt GUI application):
```
docker run -e "DISPLAY=$DISPLAY" -v "$HOME/.Xauthority:/root/.Xauthority:ro" -v $GTDD_HOME:/root/.3dsystems --network host touch-focal Touch_Setup
```

Launch ROS2 nodes:
```
docker run -it --network host -v $GTDD_HOME:/root/.3dsystems touch-focal ros2 launch touch_control two_devices.launch.py
```