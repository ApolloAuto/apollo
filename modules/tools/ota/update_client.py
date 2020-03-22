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

import requests
import os
import sys
import urllib3
from configparser import ConfigParser
from modules.data.proto.static_info_pb2 import VehicleInfo
import common.proto_utils as proto_utils


def update():
    # setup server url
    config = ConfigParser()
    CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.ini')
    config.read(CONFIG_FILE)
    ip = config.get('Host', 'ip')
    port = config.get('Host', 'port')
    url = 'https://' + ip + ':' + port + '/update'

    # setup car info
    vehicle_info = VehicleInfo()
    VEHICLE_INFO_FILE = os.path.join(
        os.path.dirname(__file__), 'vehicle_info.pb.txt')
    try:
        proto_utils.get_pb_from_text_file(VEHICLE_INFO_FILE, vehicle_info)
    except IOError:
        print("vehicle_info.pb.txt cannot be open file.")
        exit()

    brand = VehicleInfo.Brand.Name(vehicle_info.brand)
    model = VehicleInfo.Model.Name(vehicle_info.model)
    vin = vehicle_info.vehicle_config.vehicle_id.vin
    car_info = {
        "car_type": brand + "." + model,
        "tag": sys.argv[1],
        "vin": vin,
    }

    urllib3.disable_warnings()
    CERT_FILE = os.path.join(os.path.dirname(__file__), 'ota.cert')
    r = requests.post(url, json=car_info, verify=CERT_FILE)
    if r.status_code == 200:
        print("Update successfully.")
        sys.exit(0)
    elif r.status_code == 400:
        print("Invalid Request.")
    else:
        print("Cannot connect to server.")
    sys.exit(1)


if __name__ == "__main__":
    update()
