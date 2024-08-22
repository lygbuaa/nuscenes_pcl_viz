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
YAML_CONF_PATH=${PRJ_PATH}/config.yaml
CAM_JSON_CONF_PATH=/home/hugoliu/alaska/dataset/nuscenes/data_set_noa_sgrbg12/nusc_dataset_noa_sgrbg12.json
LIDAR_JSON_CONF_PATH=/dev/shm/nuscenes/mini/nusc_dataset_noa_sgrbg12.json
TRT_MODEL_PATH=/home/hugoliu/github/nuscenes/nuscenes_pcl_viz/centerpoint/model/centerpoint_fp16.trt

cd ${PRJ_PATH}
source set_env.sh

build/lidar_encoder_bin ${YAML_CONF_PATH} ${CAM_JSON_CONF_PATH} ${LIDAR_JSON_CONF_PATH} ${TRT_MODEL_PATH}