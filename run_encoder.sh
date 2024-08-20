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

PRJ_PATH=$( find_project_root_path )
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
CONF_FILE_PATH=${PRJ_PATH}/config.yaml

cd ${PRJ_PATH}
source set_env.sh

build/lidar_encoder_bin ${CONF_FILE_PATH}