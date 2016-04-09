#!/bin/bash
#
# Workaround script to initialize environmental variable at launch, enabling development tools (non-dynamic walking).
#

echo "Running launchfile with DRC_CHEATS_ENABLED = 1"
VRC_CHEATS_ENABLED=1 roslaunch aslam_project atlas_test040816.launch
echo

