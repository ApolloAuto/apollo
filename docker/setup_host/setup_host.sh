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

# Setup core dump format.
if [ -e /proc/sys/kernel ]; then
  echo "${APOLLO_ROOT_DIR}/data/core/core_%e.%p" | \
      sudo tee /proc/sys/kernel/core_pattern
fi

# Setup ntpdate to run once per minute. Log at /var/log/syslog.
grep -q ntpdate /etc/crontab
if [ $? -ne 0 ]; then
  echo "*/1 * * * * root ntpdate -v -u us.pool.ntp.org" | \
      sudo tee -a /etc/crontab
fi

# Add udev rules.
sudo cp -r ${APOLLO_ROOT_DIR}/docker/setup_host/etc/* /etc/

# Add uvcvideo clock config.
grep -q uvcvideo /etc/modules
if [ $? -ne 0 ]; then
  echo "uvcvideo clock=realtime" | sudo tee -a /etc/modules
fi
