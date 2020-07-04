#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

[ -x "$(command -v ntpdate)" ] || sudo apt-get install -y ntpdate

REGIN="us"

if [ "$#" -ne 1 ]; then
    echo "Usage:      ./time_sync.sh <ntp_pool_regin>"
    echo "Example:    ./time_sync.sh cn                               # Set ntp pool to China server"
    echo "Default using us as ntp pool regin"
else
    REGIN=$1
fi

echo "Regin is set as: ${REGIN}, make sure this regin is consistent with list on pool.npt.org"

grep -q ntpdate /etc/crontab
if [ $? -eq 1 ]; then
    echo "*/1 * * * * root ntpdate -v -u ${REGIN}.pool.ntp.org" | sudo tee -a /etc/crontab
fi

# ntpdate running log at /var/log/syslog

sudo ntpdate -v -u ${REGIN}.pool.ntp.org
