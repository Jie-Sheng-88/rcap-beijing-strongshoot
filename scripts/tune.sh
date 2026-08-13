#!/bin/bash
echo "[TUNE] live parameter tuner for brain_node"

cd `dirname $0`
cd ..

source ./install/setup.bash

# The tuner and brain_node must use the SAME FastDDS profile. Mismatched
# transport descriptors mean the two participants never complete discovery and
# the tuner just sits on "waiting for /brain_node parameter services".
#
# The start scripts disagree about which profile that is: start.sh uses the
# vendor profile below, start_brain.sh uses ./configs/fastdds.xml. Resolve it in
# priority order, and NEVER let a failed pid lookup pick the wrong file --
# start.sh is what actually runs in matches, so its profile is the default.
VENDOR_PROFILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
REPO_PROFILE=./configs/fastdds.xml

# 1. Explicit override always wins:
#      TUNE_PROFILE=/path/to.xml ./scripts/tune.sh
#      TUNE_PROFILE=none ./scripts/tune.sh     # run with no profile at all
PROFILE="$TUNE_PROFILE"
SOURCE="TUNE_PROFILE override"

# 2. Otherwise read it back from a running brain_node. Try the full install path
#    first (that is what shows up in ps), then looser matches.
if [ -z "$PROFILE" ]; then
    for pattern in 'lib/brain/brain_node' 'brain_node --ros-args' 'brain_node'; do
        BRAIN_PID=$(pgrep -f "$pattern" | head -1)
        [ -n "$BRAIN_PID" ] && break
    done
    if [ -n "$BRAIN_PID" ] && [ -r "/proc/$BRAIN_PID/environ" ]; then
        PROFILE=$(tr '\0' '\n' < "/proc/$BRAIN_PID/environ" \
            | sed -n 's/^FASTDDS_DEFAULT_PROFILES_FILE=//p' | head -1)
        [ -z "$PROFILE" ] && PROFILE=none
        SOURCE="running brain_node (pid $BRAIN_PID)"
    elif [ -n "$BRAIN_PID" ]; then
        echo "[TUNE] brain_node is pid $BRAIN_PID but /proc/$BRAIN_PID/environ is unreadable" \
             "(owner $(stat -c %U /proc/$BRAIN_PID 2>/dev/null || echo unknown), you $(id -un))"
    else
        echo "[TUNE] no brain_node process matched; falling back to the start.sh profile"
        echo "[TUNE]   check with: pgrep -a -f brain_node   /   tail -50 brain.log"
    fi
fi

# 3. Fall back to whichever profile file actually exists on this machine.
if [ -z "$PROFILE" ]; then
    if [ -r "$VENDOR_PROFILE" ]; then
        PROFILE="$VENDOR_PROFILE"
        SOURCE="start.sh default"
    else
        PROFILE="$REPO_PROFILE"
        SOURCE="start_brain.sh default"
    fi
fi

unset FASTRTPS_DEFAULT_PROFILES_FILE
if [ "$PROFILE" = "none" ]; then
    echo "[TUNE] profile: none ($SOURCE)"
    unset FASTDDS_DEFAULT_PROFILES_FILE
else
    if [ ! -r "$PROFILE" ]; then
        # FastDDS ignores an unreadable profile silently and uses builtin
        # transports, which looks identical to a hang. Say so instead.
        echo "[TUNE] WARNING: profile '$PROFILE' is missing or unreadable;" \
             "FastDDS will silently ignore it"
    fi
    echo "[TUNE] profile: $PROFILE ($SOURCE)"
    export FASTDDS_DEFAULT_PROFILES_FILE="$PROFILE"
fi

python3 ./scripts/tune.py "$@"
