#!/usr/bin/env python

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
"""OTA verify client"""

import requests
import os
import sys
from ConfigParser import ConfigParser
from modules.data.proto.task_pb2 import VehicleInfo
import common.proto_utils as proto_utils
sys.path.append('/home/caros/secure_upgrade/python')
import secure_upgrade_export as sec_api

root_config_path = "/home/caros/secure_upgrade/config/secure_config.json"
returnCode = sec_api.init_secure_upgrade(root_config_path)
if returnCode == True:
    print 'Security environment init successfully!'
else:
    print 'Security environment init fail!'
    sys.exit(1)

def verify():
    # generate orig update package
    token_file_name = os.environ['HOME'] + '/.cache/apollo_update/auth_token'
    with open(token_file_name, 'r') as token_file:
        auth_token = token_file.read()
    sec_package = os.environ['HOME'] + '/.cache/sec_apollo_release.tar.gz'
    orig_package = os.environ['HOME'] + '/.cache/apollo_release.tar.gz'
    returnCode = sec_api.sec_upgrade_verify_package(auth_token,
                                                    sec_package,
                                                    orig_package)
    if returnCode == True:
        print 'verify package  successfully!'
        sys.exit(0)
    else:
        print 'verify package failed!!!'
        sys.exit(1)

if __name__ == "__main__":
    verify()
