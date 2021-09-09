#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"
source "$ROOT_DIR/docker/common.sh"


if ! docker ps --format "{{.Names}}" | grep "^$CONTAINER_NAME$" > /dev/null 2>&1; then
  echo "Bringing up container"
  "$ROOT_DIR/docker/start.sh"
fi

echo "Rebuilding customer interface files..."
run_in_dev_container "mkdir -p /simian/build/ && cd /simian/build && cmake ../ && make -j12"
echo "Finished building and updating customer interface..."
