#!/bin/bash
echo "[TUNE] live parameter tuner for brain_node"

cd `dirname $0`
cd ..

source ./install/setup.bash

# The tuner and brain_node must use the SAME FastDDS profile. Mismatched
# transport descriptors mean the two participants never complete discovery and
# the tuner just sits on "waiting for /brain_node parameter services".
#
# The start scripts disagree about which profile that is: start_brain.sh uses
# ./configs/fastdds.xml, start.sh uses /opt/booster/BoosterRos2/
# fastdds_profile_udp_only.xml. Rather than hardcode a third copy that drifts,
# read the profile back out of the running brain_node.
BRAIN_PID=$(pgrep -x brain_node | head -1)

unset FASTRTPS_DEFAULT_PROFILES_FILE

if [ -n "$BRAIN_PID" ] && [ -r "/proc/$BRAIN_PID/environ" ]; then
    BRAIN_PROFILE=$(tr '\0' '\n' < "/proc/$BRAIN_PID/environ" \
        | sed -n 's/^FASTDDS_DEFAULT_PROFILES_FILE=//p' | head -1)
    if [ -n "$BRAIN_PROFILE" ]; then
        echo "[TUNE] matching brain_node (pid $BRAIN_PID) profile: $BRAIN_PROFILE"
        export FASTDDS_DEFAULT_PROFILES_FILE="$BRAIN_PROFILE"
    else
        # brain_node runs with no profile; the tuner must not set one either.
        echo "[TUNE] brain_node (pid $BRAIN_PID) runs with no FastDDS profile, matching that"
        unset FASTDDS_DEFAULT_PROFILES_FILE
    fi
else
    echo "[TUNE] brain_node not running (or /proc unreadable), assuming ./configs/fastdds.xml"
    export FASTDDS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml
fi

python3 ./scripts/tune.py "$@"
