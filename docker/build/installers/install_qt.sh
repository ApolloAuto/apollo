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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

QT_VERSION_A=5.9
QT_VERSION_B=5.9.8

ARCH=$(uname -m)

wget https://download.qt.io/archive/qt/${QT_VERSION_A}/${QT_VERSION_B}/qt-opensource-linux-x64-${QT_VERSION_B}.run

chmod +x qt-opensource-linux-x64-${QT_VERSION_B}.run

    # The '-platform' flag causes a message to stdout "Unknown option: p, l, a, t, f, o, r, m": message is incorrectly printed (it's a bug). The command still succeeds.
    # https://stackoverflow.com/a/34032216/1158977

    # the below error can be ignored since Ubuntu 14 does not have sslv2
    # qt.network.ssl: QSslSocket: cannot resolve SSLv2_client_method
    # qt.network.ssl: QSslSocket: cannot resolve SSLv2_server_method
./qt-opensource-linux-x64-${QT_VERSION_B}.run --script qt-noninteractive.qs  -platform minimal

mkdir /usr/local/Qt$QT_VERSION_B
ln -s /qt/$QT_VERSION_B /usr/local/Qt$QT_VERSION_B/$QT_VERSION_A

    # clean up
rm qt-opensource-linux-x64-${QT_VERSION_B}.run
rm -rf /usr/local/Qt$QT_VERSION_B/{Docs,Examples,Extras,Tools}
