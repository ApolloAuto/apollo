"""OTA update client"""

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
import os.path
import os
from ConfigParser import ConfigParser
from modules.data.proto.task_pb2 import VehicleInfo
import common.proto_utils as proto_utils


def update():
    if not os.path.exists('update.ini'):
        print "No update found!"
        exit()
        
    config = ConfigParser()
    config.read('update.ini')
    update_tag = config.get('Update', 'tag')
    usr_name = os.environ['DOCKER_USER']
    cmd = 'ssh ' + usr_name + '@localhost' + ' docker pull ' + update_tag
    code = os.system(cmd)
    if code != 0:
        print "Update fail!"
        exit()

    # setup server url
    ip = config.get('Host', 'ip')
    port = config.get('Host', 'port')
    url = 'http://' + ip + ':' + port + '/update'
    
    # setup car info
    vehicle_info = VehicleInfo()
    try:
        proto_utils.get_pb_from_text_file("vehicle_info.pb.txt", vehicle_info)
    except IOError:
        print "vehicle_info.pb.txt cannot be open file."
        exit()

    brand = VehicleInfo.Brand.Name(vehicle_info.brand)
    model = VehicleInfo.Model.Name(vehicle_info.model)
    vin = vehicle_info.license.vin
    car_info = {
        "car_type" : brand + "." + model,
        "tag" : update_tag,
        "vin" : vin,
    }

    r = requests.post(url, data=car_info)
    if r.status_code == 200:
        os.system("rm update.ini")
        print "Update successfully."
    elif r.status_code == 400:
        print "Invalid Request."
    else:
        print "Cannot connect to server."

if __name__ == "__main__":
    update()
