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

# Fail on first error.
set -e

# Create required soft links.
ln -rs /usr/lib/x86_64-linux-gnu/libprofiler.so.0 /usr/lib/libprofiler.so
ln -rs /usr/lib/x86_64-linux-gnu/libtcmalloc_and_profiler.so.4 /usr/lib/libtcmalloc_and_profiler.so
# https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
ln -s -f /bin/true /usr/bin/chfn
