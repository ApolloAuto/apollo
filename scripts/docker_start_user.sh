#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

##===========================================================##

# TODO(storypku):
# Save these rc files to /opt/apollo/rcfiles in docker build stage.
# and copied to user's `$HOME` directory with docker_start_user.sh
# Ref: https://serverfault.com/questions/72476/clean-way-to-write-complex-multi-line-string-to-a-variable
IFS='' read -r -d '' BASHRC_TEXT << EOF
export PATH="\$PATH:/apollo/scripts:/usr/local/miniconda/bin"
if [ -e "/apollo/scripts/apollo_base.sh" ]; then
  source /apollo/scripts/apollo_base.sh
fi
ulimit -c unlimited
EOF

IFS='' read -r -d '' LCOVRC_TEXT << EOF
genhtml_branch_coverage = 1
lcov_branch_coverage = 1
EOF

##===========================================================##

function _create_user_account() {
    local user_name="$1"
    local uid="$2"
    local group_name="$3"
    local gid="$4"
    addgroup --gid "${gid}" "${group_name}"

    adduser --disabled-password --force-badname --gecos '' \
            "${user_name}" --uid "${uid}" --gid "${gid}" # 2>/dev/null

    usermod -aG sudo "${user_name}"
    return 0
}

function setup_user_bashrc() {
    local uid="$1"
    local gid="$2"
    local user_home="/home/$3"
    cp -rf /etc/skel/.{profile,bash*} "${user_home}"
    # Set user files ownership to current user, such as .bashrc, .profile, etc.
    echo "${BASHRC_TEXT}" >> "${user_home}/.bashrc"
    echo "${LCOVRC_TEXT}" > "${user_home}/.lcovrc"
    chown -R "${uid}:${gid}" "${user_home}"
}

function setup_user_account() {
    local user_name="$1"
    local uid="$2"
    local group_name="$3"
    local gid="$4"
    _create_user_account "$@"
    setup_user_bashrc "${uid}" "${gid}" "${user_name}"
}

function grant_device_permissions() {
    # setup GPS device
    [ -e /dev/novatel0 ] && chmod a+rw /dev/novatel0
    [ -e /dev/novatel1 ] && chmod a+rw /dev/novatel1
    [ -e /dev/novatel2 ] && chmod a+rw /dev/novatel2

    [ -e /dev/ttyACM0 ]  && chmod a+rw /dev/ttyACM0

    # setup camera device
    [ -e /dev/camera/obstacle ]      && chmod a+rw /dev/camera/obstacle
    [ -e /dev/camera/trafficlights ] && chmod a+rw /dev/camera/trafficlights
}

##===================== Main ==============================##
function main() {
    local user_name="$1"
    local uid="$2"
    local group_name="$3"
    local gid="$4"

    if [ "${uid}" != "${gid}" ]; then
        echo "Warning: uid(${uid}) != gid(${gid}) found."
    fi
    if [ "${user_name}" != "${group_name}" ]; then
        echo "Warning: user_name(${user_name}) != group_name(${group_name}) found."
    fi
    setup_user_account "$@"
    grant_device_permissions
}

main "${DOCKER_USER}" "${DOCKER_USER_ID}" "${DOCKER_GRP}" "${DOCKER_GRP_ID}"

