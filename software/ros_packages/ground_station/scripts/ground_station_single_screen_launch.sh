#!/usr/bin/env bash

current_folder_name="scripts"
current_folder_name_length=`expr length $current_folder_name`

launch_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
launch_dir_length=`expr length $launch_dir`

launch_dir_length_without_current_folder=$(($launch_dir_length-$current_folder_name_length))

script_launch_path="${launch_dir:0:$launch_dir_length_without_current_folder}/src"
cd $script_launch_path

cp ~/key .

sleep 1

# there needs to be an initial display value
export DISPLAY=:0
# launch Xephyr in background to emulate two 1920x1080 displays
Xephyr +xinerama -screen 1920x1080 -screen 1920x1080 -ac :1 &
# change the display variable to allow connecting to Xephyr
export DISPLAY=:1

python ground_station.py
