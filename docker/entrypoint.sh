#!/usr/bin/env bash
set -euo pipefail

# Each invocation owns its temporary GUI/log directories, including when a
# caller selects a different unprivileged UID for local display access.
onrobot_session_dir=$(mktemp -d /tmp/onrobot-session.XXXXXX)
export ROS_LOG_DIR="${ROS_LOG_DIR:-$onrobot_session_dir/log}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-$onrobot_session_dir/runtime}"
mkdir -p -m 700 -- "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 -- "$onrobot_session_dir"

# ROS environment hooks do not all support nounset.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/onrobot/setup.bash
set -u
exec "$@"
