#!/bin/bash
#
# Workaround script to initialize environmental variable and atlas mode, allowing non-dynamic walking.
#
#echo "Setting multisense_sl frames per second"
#echo
rostopic pub --once /multisense_sl/set_fps std_msgs/Float64 '{data: 10}'
echo
sleep 10    #timer to ensure gazebo has finished launching
echo "Publishing /atlas/mode to pid_stand"
echo
rostopic pub --once /atlas/mode std_msgs/String "pid_stand"
echo
echo "Publishing /atlas/mode to pinned to keep feet off ground"
echo
rostopic pub --once /atlas/mode std_msgs/String "pinned"
echo


#for test
#echo "Commanding velocity"
#rostopic pub -r 10 atlas/cmd_vel geometry_msgs/Twist '{ linear: { x: 0.5, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.5 } }'
#echo
