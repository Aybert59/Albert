#!/bin/bash

echo "starting albert"

source /root/albert_ws/devel/setup.bash
PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.10/dist-packages
roslaunch /root/albert_ws/src/albert_launch/launch/albert.launch

