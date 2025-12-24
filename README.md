# hex_arm


## Warning!!!!
This repository has been discontinued for maintenance. Please proceed to: [hex_device](https://github.com/hexfellow/hex_device)

## Overview
This is a ROS packdge that provides ROS interface for hex arm.
## Prerequisites & Usage
Coming soon...
## Publish
| Topic           | Msg Type                     | Description               |
| --------------- | ---------------------------- | ------------------------- |
| /`ws_down`      | `std_msgs/UInt8MultiArray`   | Message from ws           |
| /`joint_states` | `sensor_msgs/JointState`     | Every single motor status |
| /`json_feedback` | `xpkg_arm_msgs/XmsgArmJointParamList` | Feedback of the joint parameters|

## Subscribe
| Topic         | Msg Type                   | Description           |
| ------------- | -------------------------- | --------------------- |
| /`ws_up`      | `std_msgs/UInt8MultiArray` | Message to send to ws |
| /`joints_cmd` | `sensor_msgs/JointState`   | Joint control command |