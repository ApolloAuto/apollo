/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/map/tools/map_datachecker/client/exception_handler.h"

#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace apollo {
namespace hdmap {

int ExceptionHandler::ExceptionHandlerFun(ErrorCode error_code) {
  int ret = 0;
  switch (error_code) {
    case ErrorCode::SUCCESS:
      AINFO << "ErrorCode::SUCCESS";
      fprintf(USER_STREAM, "SUCCESS\n");
      break;
    case ErrorCode::ERROR_REPEATED_START:
      AINFO << "ErrorCode::ERROR_CHECK_BEFORE_START";
      fprintf(USER_STREAM,
              "Do not start repeated. This request will be ignored\n");
      break;
    case ErrorCode::ERROR_CHECK_BEFORE_START:
      AINFO << "ErrorCode::ERROR_CHECK_BEFORE_START";
      fprintf(USER_STREAM,
              "Start command should be called before check. This request will "
              "be ignored\n");
      break;
    case ErrorCode::ERROR_REQUEST:
      AINFO << "ErrorCode::ERROR_REQUEST";
      fprintf(USER_STREAM, "Request error. This request will be ignored\n");
      break;
    case ErrorCode::ERROR_GNSS_SIGNAL_FAIL:
      AINFO << "ErrorCode::ERROR_GNSS_SIGNAL_FAIL."
            << "Please check if area is spacious";
      fprintf(USER_STREAM,
              "ERROR: GNSS signal do not meet the requirements, please make "
              "sure area is spacious\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_VERIFY_NO_GNSSPOS:
      AINFO << "ErrorCode::ERROR_VERIFY_NO_GNSSPOS."
            << "Please check if channel /apollo/sensor/gnss/best_pose exists "
               "in system";
      fprintf(USER_STREAM,
              "ERROR:System has no channel /apollo/sensor/gnss/best_pose, you "
              "may need to reboot system\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_NOT_STATIC:
      AINFO << "ErrorCode::ERROR_NOT_STATIC. Please keep the car still";
      fprintf(USER_STREAM, "ERROR:Please keep the car still\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_NOT_EIGHT_ROUTE:
      AINFO << "ErrorCode::ERROR_NOT_EIGHT_ROUTE. "
            << "Please keep the car 8-like maneuver";
      fprintf(USER_STREAM, "WARNING:Please keep the car 8-like maneuver\n");
      break;
    case ErrorCode::ERROR_LOOPS_NOT_REACHED:
      AINFO << "ErrorCode.ERROR_LOOPS_NOT_REACHED";
      fprintf(USER_STREAM,
              "WARNING:Collection time do not meet the requirements. "
              "Supplementary data collection may be required\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK:
      AINFO << "ErrorCode.ERROR_CHANNEL_VERIFY_TOPIC_LACK";
      fprintf(USER_STREAM, "ERROR: Missing topic\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL:
      AINFO << "ErrorCode.ERROR_CHANNEL_VERIFY_RATES_ABNORMAL";
      fprintf(USER_STREAM, "ERROR: Missing topic\n");
      ret = -1;
      break;
    case ErrorCode::ERROR_VERIFY_NO_RECORDERS:
      AINFO << "ErrorCode.ERROR_VERIFY_NO_RECORDERS";
      fprintf(USER_STREAM, "ERROR: Missing record\n");
      ret = -1;
      break;
    default:
      AINFO << "This branch should never be reached. If this happened, please "
               "open an issue. code: "
            << error_code;
      fprintf(USER_STREAM,
              "ERROR:This branch should never be reached. If this happened, "
              "please open an issue\n");
      break;
  }
  return ret;
}

}  // namespace hdmap
}  // namespace apollo
