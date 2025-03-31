#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

set -euo pipefail

# 1. Setup core dump format.
CORE_DIR="${APOLLO_ROOT_DIR}/data/core"
if [ ! -d "${CORE_DIR}" ]; then
  sudo mkdir -p "${CORE_DIR}"
fi
sudo tee /etc/sysctl.d/99-core-dump.conf > /dev/null <<EOF
kernel.core_pattern = ${CORE_DIR}/core_%e.%p
EOF
# Apply sysctl configuration to make it take effect immediately.
sudo sysctl -p /etc/sysctl.d/99-core-dump.conf

# 2. Setup ntpdate to run once per minute. Log at /var/log/syslog.
if systemctl list-unit-files | grep -q "^systemd-timesyncd.service"; then
  echo "Enable and start the systemd-timesyncd time synchronization service..."
  sudo systemctl enable systemd-timesyncd
  sudo systemctl start systemd-timesyncd
else
  echo "systemd-timesyncd service not found!"
fi

# 3. Add udev rules.
sudo cp -r ${APOLLO_ROOT_DIR}/docker/setup_host/etc/udev/rules.d/* /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. Add uvcvideo clock config.
sudo tee /etc/modprobe.d/uvcvideo.conf > /dev/null <<EOF
options uvcvideo clock=realtime
EOF
# If the uvcvideo module is already loaded, unload and reload it for the configuration to take effect.
if lsmod | grep -q uvcvideo; then
  sudo modprobe -r uvcvideo
fi
sudo modprobe uvcvideo

echo "Setup host machine successfully!"
