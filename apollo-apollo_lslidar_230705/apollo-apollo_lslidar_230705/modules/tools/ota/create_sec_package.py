#!/usr/bin/env python3

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

import secure_upgrade_export as sec_api
import os
import sys
sys.path.append('/home/caros/secure_upgrade/python')

root_config_path = "/home/caros/secure_upgrade/config/secure_config.json"
ret = sec_api.init_secure_upgrade(root_config_path)
if ret is True:
    print('Security environment init successfully!')
else:
    print('Security environment init failed!')
    exit(1)

homedir = os.environ['HOME']
release_tgz = homedir + '/.cache/apollo_release.tar.gz'
sec_release_tgz = homedir + '/.cache/sec_apollo_release.tar.gz'
package_token = homedir + '/.cache/package_token'

ret = sec_api.sec_upgrade_get_package(
    release_tgz, sec_release_tgz, package_token)
if ret is True:
    print('Security package generated successfully!')
else:
    print('Security package generated failed!')
