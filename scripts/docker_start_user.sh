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

##========= General Purpose Functions ======================##
function gid_by_group_name() {
    local group_name="$1"
    awk -v grp="${group_name}"  -F':'   '{
        if ($1 == grp) printf("%s", $3);
    }' /etc/group
}

function user_name_by_uid() {
    local uid="$1"
    awk -v uid="${uid}" -F':'   '{
        if ($3 == uid) printf("%s", $1);
    }' /etc/passwd
}

##===========================================================##

# TODO(storypku):
# Save these rc files to /opt/apollo/misc when docker build image
# and copied to user's `$HOME` directory from there.
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

function _create_user_account_if_none_exist() {
    local user_name="$1"
    local uid="$2"
    local group_name="$3"
    local gid="$4"
    # man 5 passwd
    if grep -q "^${user_name}:x:" /etc/passwd ; then
        echo "Oops, USER ${user_name} already exist. Exiting..."
        return 1
    fi
    # man 5 group
    if grep -q "^${group_name}:x:" /etc/group; then # group name already exist
        echo "Group ${group_name} already exist."
    elif grep -q ":x:${gid}:" /etc/group; then # gid taken while group_name available
        echo "Group id ${gid} already taken. Create group without gid specified."
        addgroup "${group_name}"
        gid=$(gid_by_group_name "${group_name}")
    else
        addgroup --gid "${gid}" "${group_name}"
    fi

    # Create user
    somebody=$(user_name_by_uid "${uid}")
    if [ -z "${somebody}" ]; then # uid not taken
        adduser --disabled-password --force-badname --gecos '' \
            "${user_name}" --uid "${uid}" --gid "${gid}" # 2>/dev/null
    else # uid already taken
        adduser --disabled-password --force-badname --gecos '' \
            "${user_name}" --gid "${gid}" # 2>/dev/null
    fi

    usermod -aG sudo "${user_name}"
    return 0
}

function setup_user_bashrc() {
    local user_name="$1"
    local group_name="$2"
    local user_home="/home/$1"
    # TODO(storypku): perform copy operations in `installers/install_user.sh`
    # for user `apollo` when docker build image
    # TODO-BEGIN
    cp -rf /etc/skel/.* "${user_home}"
    # Set user files ownership to current user, such as .bashrc, .profile, etc.
    echo "${BASHRC_TEXT}" >> ${user_home}/.bashrc
    echo "${LCOVRC_TEXT}" > ${user_home}/.lcovrc
    # TODO-END
    chown -R ${user_name}:${group_name} "${user_home}"
}

function setup_user_account() {
    local user_name="$1"
    local uid="$2"
    local group_name="$3"
    local gid="$4"
    # USER apollo has already been created by `installers/install_user.sh`
    _create_user_account_if_none_exist "$@"
    setup_user_bashrc "${user_name}" "${group_name}"
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

