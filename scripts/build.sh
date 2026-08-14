#!/bin/bash

cd `dirname $0`
cd ..

unset VIRTUAL_ENV
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E 'venv/bin/?$' | paste -sd:)

export PYTHONWARNINGS="ignore::DeprecationWarning,ignore::UserWarning,ignore::FutureWarning"

colcon build  --symlink-install  --parallel-workers $(nproc) "$@"
# colcon build --parallel-workers $(nproc) "$@"

espeak "build complete" >/dev/null 2>&1 || echo "Build complete"
