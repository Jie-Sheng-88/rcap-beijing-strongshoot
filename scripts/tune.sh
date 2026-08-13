#!/bin/bash
echo "[TUNE] live parameter tuner for brain_node"

cd `dirname $0`
cd ..

# Must match start_brain.sh. configs/fastdds.xml whitelists the network
# interfaces; without it this process will not discover brain_node at all and
# the tuner just sits waiting.
source ./install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTDDS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

python3 ./scripts/tune.py "$@"
