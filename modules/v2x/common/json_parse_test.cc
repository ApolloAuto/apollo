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

#include "modules/v2x/common/json_parse.h"

#include <fstream>

#include "gtest/gtest.h"

#include "modules/v2x/proto/spat_data.pb.h"
namespace apollo {
namespace v2x {

using Json = nlohmann::json;

TEST(JsonUtilTest, toprototest) {
    char buffer[2048];
    std::ifstream in("/apollo/data/udp.json");
    if (!in.is_open()) {
        std::cout << "Error opening file";
        exit(1);
    }
    in.get(buffer, 2048, '\0');
    std::cout << "json" << buffer << std::endl;
    apollo::v2x::Spat *tf_proto = new apollo::v2x::Spat();
    std::string str = buffer;
    auto res = JsonParse::toProto(str, tf_proto, "spat");
    std::cout << res << tf_proto->DebugString();
}

}  // namespace v2x
}  // namespace apollo
