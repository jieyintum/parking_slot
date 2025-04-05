#! /bin/bash
SCRIPTPATH="$( cd -- "$(dirname -- "$0")" >/dev/null 2>&1 ; pwd -P )"
NODE_PATH=$(dirname $SCRIPTPATH)
LIB_PATH=$NODE_PATH/lib
CONFIG_PATH=$NODE_PATH/config
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIB_PATH}
cd ${SCRIPTPATH} 

#echo $CONFIG_PATH
$LIB_PATH/fusion_pse --ros-args $DDS_ARGS --params-file ${CONFIG_PATH}/fusion_pse.yaml

#--params-file $CONFIG_PATH/dead_reckoning.yaml
