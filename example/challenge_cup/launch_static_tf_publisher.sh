set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

ros2 run robo_orchard_data_ros2 tf_publisher \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/static_tf_cfg.json"
