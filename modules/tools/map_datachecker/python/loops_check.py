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
import yaml
import grpc
import exception_handler
script_path = os.path.dirname(os.path.realpath(__file__))
apollo_root = os.path.join(script_path, "../../../..")
pb_path = os.path.join(apollo_root, "modules/tools/map_datachecker/py_proto/")
sys.path.append(pb_path)
import collection_service_pb2_grpc
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
        self._exception_handler = exception_handler.ExceptionHandler.exception_handler
        
    def _get_range(self):
        time_file = os.path.join(script_path, self._conf['time_flag_file'])
        with open(time_file, "r") as fp:
            lines = fp.readlines()
            record_count = len(lines)
            if ( record_count % 2 ) != 0:
                logging.error("record_count should be even number")
                print("The command start and stop should be appear in pairs", file=sys.stderr)
                return [-1, None]
            time_range = []
            for i in range(0, record_count, 2):
                s = lines[i].strip().split()
                start_time = float(s[0])
                if s[1] != "start":
                    logging.error("state machine was broken")
                    print("The command start and stop should be appear in pairs", file=sys.stderr)
                    return [-1, None]
                s = lines[i+1].strip().split()
                end_time = float(s[0])
                if s[1] != "stop":
                    logging.error("state machine was broken")
                    print("The command start and stop should be appear in pairs", file=sys.stderr)
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
                return [-1, None]
        return [0, response]

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
        result = None
        while True:
            [ret, result] = self._check()
            if ret != 0:
                logging.error("loops check check failed")
                break
            logging.info("loops check check progress: %f" % result.progress)
            if abs(result.progress - 1.0) < 1e-8:
                break
        if ret != 0:
            return [-1, None]
        [ret, _] = self._stop()
        if ret != 0:
            logging.error("loops check stop failed")
            return [-1, None]
        return [0, result]

if __name__ == "__main__":
    loops_checker = LoopsChecker("127.0.0.1", "50100", "client.yaml")

