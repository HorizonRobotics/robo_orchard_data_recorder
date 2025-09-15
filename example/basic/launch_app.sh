set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

robo-orchard-recorder-app \
    --server.address localhost \
    -- \
    --launch-config $SCRIPT_DIR/app_launch_cfg.json \
    $1
