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
import glob
import grpc
import threading
import multiprocessing
import yaml
import time
script_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(script_path, 'py_proto'))
import collection_service_pb2_grpc
import collection_service_pb2
import collection_check_message_pb2
import collection_error_code_pb2 as ErrorCode

class ChannelChecker:
    def __init__(self, conf_file):
        self._conf_file = conf_file
        with open(conf_file, 'r') as fp:
            self._conf = yaml.load(fp.read())["channel_check"]
            self._host = self._conf['grpc_host']
            self._port = self._conf['grpc_port']
        channel = grpc.insecure_channel(self._host + ':' + self._port)
        self.stub = collection_service_pb2_grpc.CollectionCheckerServiceStub(channel)

    def _exception_handler(self, error_code):
        if error_code == ErrorCode.SUCCESS:
            logging.info("ErrorCode.SUCCESS, ignore")
        if error_code == ErrorCode.ERROR_REPEATED_START:
            logging.warn("ErrorCode.ERROR_REPEATED_START, ignore")
            return 0
        if error_code == ErrorCode.ERROR_CHECK_BEFORE_START:
            logging.warn("ErrorCode.ERROR_CHECK_BEFORE_START")
            return 0
        if error_code == ErrorCode.ERROR_VERIFY_NO_RECORDERS:
            logging.error("ErrorCode.ERROR_VERIFY_NO_RECORDERS")
            return -1
        if error_code == ErrorCode.ERROR_CHANNEL_VERIFY_TOPIC_LACK:
            logging.error("ErrorCode.ERROR_CHANNEL_VERIFY_TOPIC_LACK")
            return -1
        if error_code == ErrorCode.ERROR_CHANNEL_VERIFY_RATES_ABNORMAL:
            logging.error("ErrorCode.ERROR_CHANNEL_VERIFY_RATES_ABNORMAL")
            return -1
        logging.error("error error_code [%s]" % str(error_code))
        return -1

    def _start(self, record_path):
        request = collection_check_message_pb2.ChannelVerifyRequest(cmd=1, path=record_path)
        logging.info("channel verify start request: " + str(request))
        response = self.stub.ChannelVerify(request)
        logging.info("channel verify start response: " + str(response))
        print('[%s]' % str(type(response)))
        print('[%s]' % str(type(response.code)))
        if response.code != ErrorCode.SUCCESS:
            return self._exception_handler(response.code)
        return 0

    def _check(self):
        request = collection_check_message_pb2.ChannelVerifyRequest(cmd=2)
        logging.info("channel verify check request: " + str(request))
        response = self.stub.ChannelVerify(request)
       
        logging.info("channel verify check response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            return self._exception_handler(response.code)
        return 0

    def _stop(self):
        request = collection_check_message_pb2.ChannelVerifyRequest(cmd=3)
        logging.info("channel verify stop request: " + str(request))
        response = self.stub.ChannelVerify(request)
        logging.info("channel verify stop response: " + str(response))
        if response.code != ErrorCode.SUCCESS:
            self._exception_handler(response.code)
            return -1
        return 0

    def _periodic_check(self):
        while not os.path.exists(os.path.join(script_path, self._conf["stop_flag_file"])):
            ret = self._check()
            if ret != 0:
                break
            logging.info("channel checker sleep %d seconds" % self._conf["check_period"])
            time.sleep(self._conf["check_period"])
        logging.info("detected stop flag file, periodically checking will exit")
        


    def async_start(self, record_path):
        if not record_path or not os.path.exists(record_path):
            logging.error("record_path [%s] is not exist" % record_path)
            return -1
        record_files = []
        if os.path.isfile(record_path):
            record_files.append(record_path)
        elif os.path.isdir(record_path):
            record_files = glob.glob(record_path)
        else:
            logging.error("unrecognizable path type, path [%s]" % (record_path))
            logging.error("only support rgular file or directory")
            return -1
        if len(record_files) == 0:
            logging.error("record_path [%s] is empty" % record_path)
            return -1
        logging.info("putting channel-checking operation in the background")
        ret = self._start(record_path)
        if ret != 0:
            logging.error("start check channel failed, record_path [%s]" % record_path)
            return -1
        # thread = threading.Thread(target=self._periodic_check)
        # thread.start()
        process = multiprocessing.Process(target=self._periodic_check)
        process.start()
        logging.info("async start checking thread, main thread will exit")

#        import state_machine
#        state_machine.StateMachine()
#        state_checker = StateMachine()
#        state_checker.state_flow('record_check_start', 'start', 'record_check_start')
        return 0

    def async_stop(self):
#        import state_machine
#        state_machine.StateMachine()
#        state_checker = StateMachine()
#        ret = state_checker.state_check('record_check_stop', 'stop', 'record_check_stop')
#        if ret != 0:
#            logging.error("stop failed, do not find state: [record_check_stop]")
#            return -1
#        self._stop()
#        state_checker.state_flow('record_check_stop', 'stop', 'record_check_stop')
        open(os.path.join(script_path, self._conf["stop_flag_file"]), 'w').close()
        self._stop()
        return 0
        
if __name__ == "__main__":
    channel_checker = ChannelChecker("127.0.0.1", "50100", "client.yaml")
    channel_checker.async_start("/apollo/data/20190517135347.record.00150")
    

