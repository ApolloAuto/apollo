/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/account/account_gflags.h"

DEFINE_string(dreamview_account_info_topic, "/apollo/dreamview/account",
              "Dreamview Account Info Topic");
DEFINE_string(dreamview_account_server_api_entry, "https://studio.apollo.auto",
              "Studio Account API Server address");
DEFINE_int32(dreamview_curl_ssl_verify_host, 1L, "curl ssl verify host");
DEFINE_int32(dreamview_curl_ssl_verify_peer, 1L, "curl ssl verify peer");
DEFINE_int32(dreamview_curl_verbose, 0L, "curl verbose");
