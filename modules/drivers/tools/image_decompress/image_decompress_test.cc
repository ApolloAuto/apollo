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
#include "modules/drivers/tools/image_decompress/image_decompress.h"

#include "cyber/init.h"
#include "gtest/gtest.h"

namespace apollo {
namespace image_decompress {

TEST(ImageDecompressComponentTest, Init) {
  cyber::Init("image_decompress_component_test");
  ImageDecompressComponent component;
}

}  // namespace image_decompress
}  // namespace apollo
