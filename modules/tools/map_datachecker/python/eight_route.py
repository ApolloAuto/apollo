#!/usr/bin/env python

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
from __future__ import print_function
import os
import sys
import logging
import time
import grpc
import yaml
import exception_handler
script_path = os.path.dirname(os.path.realpath(__file__))
apollo_root = os.path.join(script_path, "../../../..")
pb_path = os.path.join(apollo_root, "py_proto/modules/map/tools/map_datachecker/proto/")
sys.path.append(pb_path)
import collection_service_pb2_grpc
import collection_check_message_pb2
import collection_error_code_pb2 as ErrorCode

class EightRoute:
    def __init__(self, conf_file):
        self._conf_file = conf_file
        with open(conf_file, 'r') as fp:
            self._conf = yaml.load(fp.read())["eight_route"]
            self._host = self._conf['grpc_host']
            self._port = self._conf['grpc_port']
        channel = grpc.insecure_channel(self._host + ':' + self._port)
        self.stub = collection_service_pb2_grpc.CollectionCheckerServiceStub(channel)
        self._exception_handler = exception_handler.ExceptionHandler.exception_handler

    def _start(self):
        request = collection_check_message_pb2.EightRouteRequest(cmd=1)
        logging.info("eight route start request: " + str(request))
        response = self.stub.EightRoute(request)
        logging.info("eight route start response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            return self._exception_handler(response.code)
        return 0
 
    def _check(self):
        request = collection_check_message_pb2.EightRouteRequest(cmd=2)
        logging.info("eight route check request: " + str(request))
        response = self.stub.EightRoute(request)
        logging.info("eight route check response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def _stop(self):
        request = collection_check_message_pb2.EightRouteRequest(cmd=3)
        logging.info("eight route stop request: " + str(request))
        response = self.stub.EightRoute(request)
        logging.info("eight route stop response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def sync_start(self):
        ret = self._start()
        if ret != 0:
            logging.error("eight route start failed")
            return -1
        while True:
            [ret, progress] = self._check()
            if ret != 0:
                logging.error("eight route check failed")
                break
            logging.info("eight route check progress: %f" % progress)
            print("eight route progress: %f" % progress, file=sys.stderr)
            if abs(progress - 1.0) < 1e-8:
                break
            time.sleep(self._conf["check_period"])
        if ret != 0:
            return -1
        [ret, _] = self._stop()
        if ret != 0:
            logging.error("eight route stop failed")
            return -1
        return 0
    
    def sync_stop(self):
        [ret, _] = self._stop()
        if ret != 0:
            logging.error("eight route stop failed")
            return -1
        return 0

if __name__ == "__main__":
    eight_route = EightRoute("127.0.0.1", "50100", "client.yaml")

