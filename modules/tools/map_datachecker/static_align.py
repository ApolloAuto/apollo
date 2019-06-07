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

import os
import sys
import logging
import yaml
import math
import time
import grpc
script_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(script_path, 'py_proto'))
import collection_service_pb2_grpc
import collection_service_pb2
import collection_check_message_pb2
import collection_error_code_pb2 as ErrorCode

class StaticAlign:
    def __init__(self, conf_file):
        self._conf_file = conf_file
        with open(conf_file, 'r') as fp:
            self._conf = yaml.load(fp.read())["static_align"]
            self._host = self._conf['grpc_host']
            self._port = self._conf['grpc_port']
        channel = grpc.insecure_channel(self._host + ':' + self._port)
        self.stub = collection_service_pb2_grpc.CollectionCheckerServiceStub(channel)

    def _exception_handler(self, error_code):
        if error_code == ErrorCode.SUCCESS:
            logging.info("ErrorCode.SUCCESS, ignore")
        elif error_code == ErrorCode.ERROR_REPEATED_START:
            logging.warn("ErrorCode.ERROR_REPEATED_START, ignore")
        elif error_code == ErrorCode.ERROR_CHECK_BEFORE_START:
            logging.warn("ErrorCode.ERROR_CHECK_BEFORE_START")
        elif error_code == ErrorCode.ERROR_REQUEST:
            logging.warn("ErrorCode.ERROR_REQUEST")
        elif error_code == ErrorCode.ERROR_GNSS_SIGNAL_FAIL:
            logging.error("ErrorCode.ERROR_GNSS_SIGNAL_FAIL")
            return -1
        elif error_code == ErrorCode.ERROR_VERIFY_NO_GNSSPOS:
            logging.error("ErrorCode.ERROR_VERIFY_NO_GNSSPOS")
            return -1
        elif error_code == ErrorCode.ERROR_NOT_STATIC:
            logging.warn("ErrorCode.ERROR_NOT_STATIC")
        else:
            logging.error("error error_code [%s]" % str(error_code))
        return 0

    def _start(self):
        request = collection_check_message_pb2.StaticAlignRequest(cmd=1)
        logging.info("static align start request: " + str(request))
        response = self.stub.StaticAlign(request)
        logging.info("static align start response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            return self._exception_handler(response.code)
        return 0
 
    def _check(self):
        request = collection_check_message_pb2.StaticAlignRequest(cmd=2)
        logging.info("static align check request: " + str(request))
        response = self.stub.StaticAlign(request)
        logging.info("static align check response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def _stop(self):
        request = collection_check_message_pb2.StaticAlignRequest(cmd=3)
        logging.info("static align stop request: " + str(request))
        response = self.stub.StaticAlign(request)
        logging.info("static align stop response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            logging.info("abnormal response: [%s]" % str(response))
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def sync_start(self):
        ret = self._start()
        if ret != 0:
            logging.error("static align start failed")
            return -1
        while True:
            [ret, progress] = self._check()
            if ret != 0:
                logging.error("static align check failed")
                break
            logging.info("static align check progress: %f" % progress)
            if abs(progress - 1.0) < 1e-8:
                break
            time.sleep(self._conf["check_period"])
        if ret != 0:
            return -1
        [ret, _] = self._stop()
        if ret != 0:
            logging.error("static align stop failed")
            return -1
        return 0

if __name__ == "__main__":
    static_align = StaticAlign("127.0.0.1", "50100", "client.yaml")

