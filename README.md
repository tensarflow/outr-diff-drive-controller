# diff-drive-controller
![](GifMaker_20200519002732215.gif)

TODO:
*  Odometry Feedback
*  Position calculation doesn't work really good
*  Must subscribe to `/cmd_vel` message and apply these velocities to the motors. (With internal PID Loop!)
*  

## Purpose

This package mainly implements kinematic calculations for the differential drive base of the OUTR. It gets specific velocity commands, calculates which motor has to turn how fast, and then drives these motors.
Encoders attached to the robot are giving incremental ticks to calculate how much they actually turned. This information is used as an internal velocity feedback but also as an odometry feedback to the server. 

(is going to) subscribes to
    
* /odom
* /cmd_vel

and publishes
* /odom
* /tf

## Dependencies

- ROS
- ESP8266WiFi
- Arduino
- PID (Arduino Library)


## Install
VS Code together with PlatformIO was used as the developement tool-chain.
To install this package on the microcontroller, select main.cpp file as the main sketch, compile and upload it with platformIO. A more general approach will be found for this. (Of course connect it with a cable :) )

## Usage

Run in terminal

```
roscore
```

Then run in seperate terminal

```
rosrun rosserial_python serial_node.py tcp
```

After that, power up the microcontroller. It should automatically connect to the ros-core.
You shoud be able to get tf transformations and visualize them in rviz.