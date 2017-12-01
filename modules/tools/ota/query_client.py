"""OTA query client"""

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
import sys
import os
from ConfigParser import ConfigParser
from modules.data.proto.task_pb2 import VehicleInfo
import common.proto_utils as proto_utils

def query():
    vehicle_info = VehicleInfo()
    VEHICLE_INFO_FILE = os.path.join(os.path.dirname(__file__), 'vehicle_info.pb.txt')
    try:
        proto_utils.get_pb_from_text_file(VEHICLE_INFO_FILE, vehicle_info)
    except IOError:
        print "vehicle_info.pb.txt cannot be open file."
        exit()
    
    # setup server url
    config = ConfigParser()
    CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.ini')
    config.read(CONFIG_FILE)
    ip = config.get('Host', 'ip')
    port = config.get('Host', 'port')
    url = 'http://' + ip + ':' + port + '/query'

    # setup car info
    brand = VehicleInfo.Brand.Name(vehicle_info.brand)
    model = VehicleInfo.Model.Name(vehicle_info.model)
    vin = vehicle_info.license.vin
    car_info = {
        "car_type" : brand + "." + model,
        "tag" : os.environ['DOCKER_IMG'],
        "vin" : vin,
    }

    r = requests.post(url, data=car_info)
    if r.status_code == 200:
        tag = r.json().get("tag")
        config.add_section('Update')
        config.set('Update', 'tag', tag)
        UPDATE_FILE = os.path.join(os.path.dirname(__file__), 'update.ini')
        with open(UPDATE_FILE, 'wb') as update_file:
            config.write(update_file)
    elif r.status_code == 204:
        print "Release is up to date."
    elif r.status_code == 400:
        print "Invalid car type."
    else:
        print "Cannot connect to server."

if __name__ == "__main__":
    query()
