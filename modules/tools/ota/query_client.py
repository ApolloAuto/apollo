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

import os
import requests
import sys

from configparser import ConfigParser
import secure_upgrade_export as sec_api
import urllib3

from modules.data.proto.static_info_pb2 import VehicleInfo
import common.proto_utils as proto_utils


sys.path.append('/home/caros/secure_upgrade/python')

root_config_path = '/home/caros/secure_upgrade/config/secure_config.json'
ret = sec_api.init_secure_upgrade(root_config_path)
if ret is False:
    print('Failed to initialize security environment!')
    sys.exit(1)


def query():
    vehicle_info = VehicleInfo()
    VEHICLE_INFO_FILE = os.path.join(
        os.path.dirname(__file__), 'vehicle_info.pb.txt')
    try:
        proto_utils.get_pb_from_text_file(VEHICLE_INFO_FILE, vehicle_info)
    except IOError:
        print('vehicle_info.pb.txt cannot be open file.')
        sys.exit(1)

    # Setup server url
    config = ConfigParser()
    CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.ini')
    config.read(CONFIG_FILE)
    ip = config.get('Host', 'ip')
    port = config.get('Host', 'port')
    url = 'https://' + ip + ':' + port + '/query'

    # Generate device token
    ret = sec_api.sec_upgrade_get_device_token()
    if ret[0] is False:
        print('Failed to get device token.')
        sys.exit(1)
    dev_token = ret[1]

    # setup car info
    brand = VehicleInfo.Brand.Name(vehicle_info.brand)
    model = VehicleInfo.Model.Name(vehicle_info.model)
    vin = vehicle_info.vehicle_config.vehicle_id.vin
    META_FILE = '/apollo/meta.ini'
    config.read(META_FILE)
    car_info = {
        "car_type": brand + "." + model,
        "tag": config.get('Release', 'tag'),
        "vin": vin,
        "token": dev_token
    }

    urllib3.disable_warnings()
    CERT_FILE = os.path.join(os.path.dirname(__file__), 'ota.cert')
    r = requests.post(url, json=car_info, verify=CERT_FILE)
    if r.status_code == 200:
        auth_token = r.json().get("auth_token")
        if auth_token == "":
            print('Cannot get authorize token!')
            sys.exit(1)
        else:
            token_file_name = os.environ['HOME'] + \
                '/.cache/apollo_update/auth_token'
            apollo_update = os.path.dirname(token_file_name)
            if not os.path.exists(apollo_update):
                os.makedirs(apollo_update)
            with open(token_file_name, 'w') as token_file:
                token_file.write(auth_token)
        tag = r.json().get("tag")
        print(tag)
        sys.exit(0)
    elif r.status_code == 204:
        print('Release is up to date.')
        sys.exit(0)
    elif r.status_code == 400:
        print('Invalid car type.')
    else:
        print('Cannot connect to server.')

    sys.exit(1)


if __name__ == '__main__':
    query()
