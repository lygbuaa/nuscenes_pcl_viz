#!/bin/bash

function find_project_root_path() {
    # echo "@i@ --> find dir: ${0}"
    this_script_dir=$( dirname -- "$0"; )
    pwd_dir=$( pwd; )
    if [ "${this_script_dir:0:1}" = "/" ]
    then
        # echo "get absolute path ${this_script_dir}" > /dev/tty
        project_root_path=${this_script_dir}"/"
    else
        # echo "get relative path ${this_script_dir}" > /dev/tty
        project_root_path=${pwd_dir}"/"${this_script_dir}"/"
    fi
    echo "${project_root_path}"
}

PRJ_ROOT_PATH=$( find_project_root_path )
echo "project_root_path: ${PRJ_ROOT_PATH}" 
cd ${PRJ_ROOT_PATH}

source /opt/ros/foxy/setup.bash
colcon build --packages-up-to pcl_rviz_bridge
