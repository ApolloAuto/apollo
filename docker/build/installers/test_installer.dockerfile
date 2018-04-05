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

FROM ubuntu:14.04

# Basic tools.
COPY pre_install.sh /tmp/pre_install.sh
RUN bash /tmp/pre_install.sh

# Set it with "--build-arg INSTALLER=<your installer>" when building.
ARG INSTALLER=not_exist

COPY ${INSTALLER} /tmp/installer.sh
RUN bash /tmp/installer.sh
