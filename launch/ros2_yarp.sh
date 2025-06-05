#!/bin/bash

# This scripts wraps ros2 run handling the SIGTERM signal - currently bugged in ros2 (https://github.com/ros2/launch/issues/666)
# In particular, the user can substitute the ros2 run command in yarpmanager with this script.
# The syntax is the same but the full path to the launch_file is needed.
# Stopping the script via yarpmanager will correctly stop the nodes. Killing the script will leave the nodes dangling.


_term() {
	echo "Cleaning up the process"
	kill -2 -${group}
}

if [ "$#" -lt 1 ]
then
	echo "Usage ros2_launch.sh [package] <launch_file> [args]"
	exit -1
fi

trap _term SIGTERM SIGINT

setsid ros2 $@ --noninteractive &

group=$!
echo "Group pid: $group"
child=$!
wait "$child"
