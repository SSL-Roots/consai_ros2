#!/bin/bash

ROBOT_ID=$1
GAIN_P=$2
GAIN_I=$3
GAIN_D=$4

ros2 service call /robot${ROBOT_ID}/enable_gain_setting std_srvs/srv/SetBool "{data: true}"
ros2 param set /robot${ROBOT_ID}/hardware_driver wheel_gain_p ${GAIN_P}
ros2 param set /robot${ROBOT_ID}/hardware_driver wheel_gain_i ${GAIN_I}
ros2 param set /robot${ROBOT_ID}/hardware_driver wheel_gain_d ${GAIN_D}
ros2 service call /robot${ROBOT_ID}/enable_gain_setting std_srvs/srv/SetBool "{data: false}"