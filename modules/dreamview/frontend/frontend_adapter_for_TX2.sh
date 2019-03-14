#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

function fix_phantomjs() {
  wget http://www.baiduapollo.club/package/js/phantomjs/bin/phantomjs -O node_modules/phantomjs-prebuilt/bin/phantomjs
  chmod +x node_modules/phantomjs-prebuilt/bin/phantomjs
  echo "INFO: fix phantomjs successfully."
}

function fix_node_sass() {
  if [ ! -d "./node_modules/node-sass/vendor" ]; then
    mkdir node_modules/node-sass/vendor
    mkdir node_modules/node-sass/vendor/linux-arm64-57
  fi
  wget http://www.baiduapollo.club/package/js/node-sass/vendor/linux-arm64-57/binding.node -O node_modules/node-sass/vendor/linux-arm64-57/binding.node
  chmod +x node_modules/node-sass/vendor/linux-arm64-57/binding.node
  echo "INFO: fix node-sass successfully."
}

function fix_optipng() {
  wget http://www.baiduapollo.club/package/js/optipng/optipng -O node_modules/optipng-bin/vendor/optipng
  chmod +x node_modules/optipng-bin/vendor/optipng
  echo "INFO: fix optipng successfully."
}

function fix_pngquant() {
  wget http://www.baiduapollo.club/package/js/pngquant/pngquant -O node_modules/pngquant-bin/vendor/pngquant
  chmod +x node_modules/pngquant-bin/vendor/pngquant
  echo "INFO: fix pngquant successfully."
}

function fix_check_arch() {
  if [ ! -f "node_modules/node-sass/lib/extensions.js" ]; then
    echo "ERROR: can not find file node_modules/node-sass/lib/extensions.js"
  fi
  sed -i "/case 'x64': return '64-bit';/a\    case 'arm64': return '64-bit';" node_modules/node-sass/lib/extensions.js
}

function main() {
  if [ ! -d "./node_modules" ]; then
    echo "ERROR: can not find directory node_modules, please execute `yarn install` first, and ignore the errors"
    exit 1
  fi
  
  fix_phantomjs
  fix_node_sass
  fix_optipng
  fix_pngquant
  fix_check_arch

  echo "INFO: fix successully!!!"
}

main $@
