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

"""
Config Parser.
"""

import sys
import os
import yaml
import logging

class ConfigParser(object):
    """This class use ConfigParser to parse config file."""
    def __init__(self):
        self.vehicle = {}
        self.organization = {}
        self.data_type = []
        self.data_args = {}
        self.task_purpose= "debug"
        self.task_data_args = {}

    def load_config(self, config_file):
        """Get configuration items from config_file."""
        yaml_cf = None
        try:
            stream = file(config_file, 'r')    
            yaml_cf = yaml.safe_load(stream) 
        except Exception as e:
            logging.error("Load config file %s encounters error, %s", config_file, str(e))
        else:
            logging.info("Load config file %s successfully", config_file)
        return yaml_cf

    def get_vehicle_from_yaml(self, yaml_config):
        """Get vehicle info from global config file."""
        vehicle = yaml_config.get('vehicle')
        if vehicle is None:
            logging.error("Config file do not contain vehicle field.")
            return -1
        vehicle_required = vehicle.get('required')
        if vehicle_required is None:
            logging.error("Config file must contain vehicle required field.")
            return -1
        try:
            self.vehicle['vehicle_id'] = vehicle_required['vehicle_id']
            self.vehicle['vehicle_tag'] = vehicle_required['vehicle_tag']
            self.vehicle['vehicle_type'] = vehicle_required['vehicle_type']
        except KeyError as e:
            logging.error("vehicle_id, vehicle_type and vehicle_tag are required, %s", str(e))
            return -1
        try:
            self.vehicle['vehicle_id'] = os.environ['CARID']
            logging.info("Get CARID from env variable successfully, CARID=%s", self.vehicle['vehicle_id'])
        except KeyError as e:
            logging.warn("Get CARID from env variable failed, read carid from conf.")
        if "optional" in vehicle:
            vehicle_optional = vehicle.get('optional')
            if vehicle_optional is not None:
                try:
                    self.vehicle['description'] = vehicle_optional['description']
                except KeyError as e:
                    logging.warn("get vehicle optional field encounters error, %s", str(e))
        logging.info("get vehicle from yaml config file successfully, vehicle=%s", self.vehicle)
        return 0

    def get_organization_from_yaml(self, yaml_config):
        """Get organization info from global config file."""
        org = yaml_config.get('organization')
        if org is None:
            logging.error("Config file do not contain organization field.")
            return -1
        org_required = org.get('required')
        if org_required is None:
            logging.error("Config file must contain organization required field.")
            return -1
        try:
            self.organization['name'] = org_required['name']
            self.organization['website'] = org_required['website']
        except KeyError as e:
            logging.error("organization name, site are required, %s", str(e))
            return -1
        if "optional" in org:
            org_optional = org.get('optional')
            if org_optional is not None:
                try:
                    self.organization['description'] = org_optional['description']
                except KeyError as e:
                    logging.warn("Get organization optional field encounters error, %s", str(e))
        logging.info("Get organization from yaml config file successfully, organization=%s",
            self.organization)
        return 0

    def get_datatype_from_yaml(self, yaml_config):
        """Get road infomation from yaml config file."""
        data = yaml_config.get('data')
        if data is None:
            logging.error("Config file do not contains data field.")
            return -1
        if "data_type" not in data:
            logging.error("Data_type is required in data field.")
            return -1
        self.data_args = data
        data_type = data.get('data_type')
        if data_type is not None:
            self.data_type = data_type
        logging.info("Get data_type from yaml config file successfully, data_type=%s", self.data_type)
        return 0
        
    def get_global_config(self, yaml_config):
        """Get global config information from yaml config file."""    
        if not self.get_vehicle_from_yaml(yaml_config) == 0:
            logging.error("get vehicle from yaml encounters error")
            return -1
        if not self.get_datatype_from_yaml(yaml_config) == 0:
            logging.error("get datatype from yaml encounters error")
            return -1
        if not self.get_organization_from_yaml(yaml_config) == 0:
            logging.error("get organization from yaml encounters error")
            return -1
        return 0
 
    def get_task_from_yaml(self, yaml_config):
        """Get task parameters from recorder.${task_purpose}.yaml."""
        try:
            task = yaml_config['task']
            task_purpose = task['task_purpose']
            task_data_args = task['task_data_args']
        except KeyError as e:
            logging.error("%s is required", str(e))
            return -1
        else:
            self.task_purpose = task_purpose
            self.task_data_args = task_data_args
        return 0
