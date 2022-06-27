/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/account/login_callback_handler.h"

#include <fstream>
#include <string>

#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

LoginCallbackHandler::LoginCallbackHandler() {}

bool LoginCallbackHandler::handleGet(CivetServer* server,
                                     struct mg_connection* conn) {
  std::string sid = "";
  CivetServer::getParam(conn, "sid", sid);

  for (const auto handler : login_success_handlers_) {
    handler(sid);
  }

  mg_printf(conn,
            "HTTP/1.1 302 Moved Temporarily\r\n"
            "Connection: close\r\n"
            "Max-Age: 0\r\n"
            "Expires: 0\r\n"
            "Cache-Control: no-cache, no-store, must-revalidate, private\r\n"
            "Pragma: no-cache\r\n"
            "Location: %s\r\n"
            "Content-Type: text/html\r\n"
            "\r\n",
            "/login_success.html");

  return true;
}

}  // namespace dreamview
}  // namespace apollo
