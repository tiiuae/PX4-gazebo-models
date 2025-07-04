#!/bin/bash

#example for run
#./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf

MODEL_PATH=$1

if [ -z $MODEL_PATH ]; then
    echo "You should specify a path to the using model"
    echo "./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf"
    exit
fi

if [ ! -f $(pwd)/build/px4_sitl_default/rootfs/gz_env.sh ]; then
    echo "You should build px4_sitl_default"
    echo "make px4_sitl_default"
    exit
fi

source $(pwd)/build/px4_sitl_default/rootfs/gz_env.sh

if [ ! -f $GZ_SIM_SYSTEM_PLUGIN_PATH/libmavlink_hitl_gazebosim.so ]; then
    echo "You should build gzsim-plugins"
    echo "make px4_sitl gzsim-plugins"
    exit
fi

spawn_model() {
    sleep 5
    MODEL=${PX4_SIM_MODELS}/${MODEL_PATH}
    NAME="HITL_Drone"
    REQUEST="sdf_filename: \"${MODEL}\", name: \"${NAME}\" pose: {position: {x: 1.01, y: 0.98, z: 0.83}}"
    gz service -s /world/default/create \
            --reqtype gz.msgs.EntityFactory \
            --reptype gz.msgs.Boolean \
            --timeout 1000 \
            --req "`echo $REQUEST`"
}

spawn_model &

WORD_PATH="${PX4_GZ_WORLDS}/default.sdf"
gz sim ${WORD_PATH} -r
