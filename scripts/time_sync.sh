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

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

function usage() {
  info "${RED}Usage${NO_COLOR}: ${BOLD}${0}${NO_COLOR} <region>"
  info "  E.g. $0 us # Sync with NTP servers for USA"
  info "  E.g. $0 cn # Sync with NTP servers for China"
}

NTPDATE_CMD="$(command -v ntpdate)"

function check_cmd_exist() {
  if [ ! -x "${NTPDATE_CMD}" ]; then
    warning "Command \"ntpdate\" not found. You can install it manually via:"
    warning "  sudo apt-get -y update && sudo apt-get -y install ntpdate"
    exit 1
  fi
}

REGION="us"

function parse_args() {
  if [ "$#" -ne 1 ]; then
    usage
    exit 1
  else
    REGION="$(echo $1 | tr 'A-Z' 'a-z')"
  fi
  info "Region set to ${REGION}."
}

# RTFM: https://www.ntppool.org/zone/@

function setup_cron_job() {
  if grep -q ntpdate /etc/crontab; then
    return
  fi
  echo "*/1 * * * * root ntpdate -v -u ${REGION}.pool.ntp.org" | sudo tee -a /etc/crontab
  sudo /etc/init.d/cron restart
  # sudo systemctl restart cron.service
}

function main() {
  check_cmd_exist
  parse_args "$@"
  setup_cron_job
  # ntpdate log goes to /var/log/syslog
  sudo ntpdate -v -u ${REGION}.pool.ntp.org
}

main "$@"
