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
# -x matches the process name; -f is the fallback for when the node is exec'd
# under a wrapper and comm/ is something else.
BRAIN_PID=$(pgrep -x brain_node | head -1)
[ -z "$BRAIN_PID" ] && BRAIN_PID=$(pgrep -f '[b]rain_node' | head -1)

unset FASTRTPS_DEFAULT_PROFILES_FILE

if [ -z "$BRAIN_PID" ]; then
    echo "[TUNE] WARNING: no brain_node process found. Is brain actually up?"
    echo "[TUNE]   pgrep -a -f brain_node"
    echo "[TUNE]   tail -50 brain.log     # start.sh backgrounds brain, crashes are silent"
    echo "[TUNE] assuming ./configs/fastdds.xml, which is WRONG if you used start.sh"
    export FASTDDS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml
elif [ ! -r "/proc/$BRAIN_PID/environ" ]; then
    # Different user (started under sudo?), so the profile cannot be read back.
    echo "[TUNE] WARNING: brain_node is pid $BRAIN_PID but /proc/$BRAIN_PID/environ is unreadable"
    echo "[TUNE]   owner: $(stat -c %U /proc/$BRAIN_PID 2>/dev/null || echo unknown), you: $(id -un)"
    echo "[TUNE] assuming ./configs/fastdds.xml, which is WRONG if you used start.sh"
    export FASTDDS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml
else
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
fi

python3 ./scripts/tune.py "$@"
