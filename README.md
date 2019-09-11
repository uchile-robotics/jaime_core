# jaime_core

## jaime installation
### Install forks:
Install dependencies:

sytem: libftdi, libusb

ROS: common_msgs, ecl, tf, nodelet, robot-state-publisher, diagnostics, dynamic-reconfigure, pcl, tf2_sensor_msgs

```
makdir forks_ws
cd forks_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/uchile-robotics/jaime_core/master/forks.rosinstall
wstool update -t src
```

### Install base:

```
makdir base_ws
cd base_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/uchile-robotics/jaime_core/master/jaime.rosinstall
wstool update -t src
```

