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
import grpc
script_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(script_path, 'py_proto'))
import collection_service_pb2_grpc
import collection_service_pb2
import collection_check_message_pb2
import collection_error_code_pb2 as ErrorCode

class LoopsChecker:
    def __init__(self, conf_file):
        self._conf_file = conf_file
        with open(conf_file, 'r') as fp:
            self._conf = yaml.load(fp.read())["loops_check"]
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
        elif error_code == ErrorCode.ERROR_LOOPS_NOT_REACHED:
            logging.error("ErrorCode.ERROR_LOOPS_NOT_REACHED")
            return -1
        else:
            logging.error("error error_code [%s]" % str(error_code))
        return 0
        
    def _get_range(self):
        time_file = os.path.join(script_path, self._conf['time_flag_file'])
        with open(time_file, "r") as fp:
            lines = fp.readlines()
            record_count = len(lines)
            if ( record_count % 2 ) != 0:
                logging.error("record_count should be even number")
                return [-1, None]
            time_range = []
            for i in range(0, record_count, 2):
                s = lines[i].strip().split()
                start_time = float(s[0])
                if s[1] != "start":
                    logging.error("state machine was broken")
                    return [-1, None]
                s = lines[i+1].strip().split()
                end_time = float(s[0])
                if s[1] != "end":
                    logging.error("state machine was broken")
                    return [-1, None]
                dic_range = {}
                dic_range["start_time"] = start_time
                dic_range["end_time"] = end_time
                time_range.append(dic_range)
        return [0, time_range]

    def _start(self, time_range):
        request = collection_check_message_pb2.LoopsVerifyRequest(cmd=1, type=collection_check_message_pb2.MAP_MAKING, range=time_range)
        logging.info("loops check start request: " + str(request))
        response = self.stub.LoopsVerify(request)
        logging.info("loops check start response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            return self._exception_handler(response.code)
        return 0
 
    def _check(self):
        request = collection_check_message_pb2.LoopsVerifyRequest(cmd=2)
        logging.info("loops check check request: " + str(request))
        response = self.stub.LoopsVerify(request)
        logging.info("loops check check response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def _stop(self):
        request = collection_check_message_pb2.LoopsVerifyRequest(cmd=3)
        logging.info("loops check stop request: " + str(request))
        response = self.stub.LoopsVerify(request)
        logging.info("loops check stop response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            ret = self._exception_handler(response.code)
            if ret != 0:
                return [-1, 0.0]
        return [0, response.progress]

    def sync_start(self):
        [ret, time_range] = self._get_range()
        if ret != 0:
            logging.error("get time range failed")
            return -1
        ret = self._start(time_range)
        if ret != 0:
            logging.error("loops check start failed")
            return -1
        while True:
            [ret, progress] = self._check()
            if ret != 0:
                logging.error("loops check check failed")
                break
            logging.info("loops check check progress: %f" % progress)
            if abs(progress - 1.0) < 1e-8:
                break
        if ret != 0:
            return -1
        [ret, _] = self._stop()
        if ret != 0:
            logging.error("loops check stop failed")
            return -1
        return 0

if __name__ == "__main__":
    loops_checker = LoopsChecker("127.0.0.1", "50100", "client.yaml")

