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
script_path = os.path.dirname(os.path.realpath(__file__))
apollo_root = os.path.join(script_path, "../../../..")
pb_path = os.path.join(apollo_root, "py_proto/modules/map/tools/map_datachecker/proto/")
sys.path.append(pb_path)
import collection_error_code_pb2 as ErrorCode

class ExceptionHandler:
    @staticmethod
    def exception_handler(self, error_code):
        if error_code == ErrorCode.SUCCESS:
            logging.info("ErrorCode.SUCCESS")
            print("SUCCESS", file=sys.stderr)
        elif error_code == ErrorCode.ERROR_REPEATED_START:
            logging.warn("ErrorCode.ERROR_REPEATED_START")
            print("Do not start repeated. This request will be ignored.", file=sys.stderr)
        elif error_code == ErrorCode.ERROR_CHECK_BEFORE_START:
            logging.warn("ErrorCode.ERROR_CHECK_BEFORE_START")
            print("Start command should be called before check. This request will be ignored.", file=sys.stderr)
        elif error_code == ErrorCode.ERROR_REQUEST:
            logging.warn("ErrorCode.ERROR_REQUEST")
            print("Request error. This request will be ignored.", file=sys.stderr)
        elif error_code == ErrorCode.ERROR_GNSS_SIGNAL_FAIL:
            logging.error("ErrorCode.ERROR_GNSS_SIGNAL_FAIL")
            logging.error("Please check if area is spacious")
            print("ERROR: GNSS signal do not meet the requirements, please make sure area is spacious", file=sys.stderr)
            return -1
        elif error_code == ErrorCode.ERROR_VERIFY_NO_GNSSPOS:
            logging.error("ErrorCode.ERROR_VERIFY_NO_GNSSPOS")
            logging.error("Please check if channel /apollo/sensor/gnss/best_pose exists in system")
            print("ERROR:System has no channel /apollo/sensor/gnss/best_pose, you may need to reboot system", file=sys.stderr)
            return -1
        elif error_code == ErrorCode.ERROR_NOT_STATIC:
            logging.error("ErrorCode.ERROR_NOT_STATIC")
            logging.error("Please keep the car still")
            print("ERROR:Please keep the car still", file=sys.stderr)
            return -1
        elif error_code == ErrorCode.ERROR_NOT_EIGHT_ROUTE:
            logging.warn("ErrorCode.ERROR_NOT_EIGHT_ROUTE")
            logging.warn("Please keep the car 8-like maneuver")
            print("WARNING:Please keep the car 8-like maneuver", file=sys.stderr)
        elif error_code == ErrorCode.ERROR_LOOPS_NOT_REACHED:
            logging.error("ErrorCode.ERROR_LOOPS_NOT_REACHED")
            print("WARNING:Collection time do not meet the requirements. Supplementary data collection may be required")
            return -1
        else:
            logging.error("error error_code [%s]" % str(error_code))
            logging.error("This branch should never be reached. If this happened, please open an issue")
            print("ERROR:This branch should never be reached. If this happened, please open an issue", file=sys.stderr)
        return 0