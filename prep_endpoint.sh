#!/bin/bash

set -xe

colcon build
sleep 1

source install/setup.bash
sleep 1

colcon build
sleep 1

source install/setup.bash
sleep 1
exit 0