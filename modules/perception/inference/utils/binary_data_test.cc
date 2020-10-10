/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/utils/binary_data.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(BinaryDataTest, test_string_io) {
  // // write and read empty string
  // {
  //   FILE *fp = fopen("./inference_test_data/utils/bd_empty", "wb");
  //   BinaryWriteString(fp, "");
  //   fclose(fp);
  // }
  // {
  //   char name[kMaxStrLen];
  //   FILE *fp = fopen("./inference_test_data/utils/bd_empty", "rb");
  //   EXPECT_EQ(BinaryReadString(fp, name), 0);
  //   fclose(fp);
  // }

  // // write and read non-empty string
  // {
  //   FILE *fp = fopen("./inference_test_data/utils/bd_non_empty", "wb");
  //   BinaryWriteString(fp, "test");
  //   fclose(fp);
  // }
  // {
  //   char name[kMaxStrLen];
  //   FILE *fp = fopen("./inference_test_data/utils/bd_non_empty", "rb");
  //   EXPECT_EQ(BinaryReadString(fp, name), 4);
  //   fclose(fp);
  // }
}

TEST(BinaryDataTest, test_blob_io) {
  // // write and read empty blob
  // {
  //   base::Blob<float> blob;
  //   FILE *fp = fopen("./inference_test_data/utils/bd_empty", "wb");
  //   BinaryWriteBlob(fp, blob);
  //   fclose(fp);
  // }
  // {
  //   char name[kMaxStrLen];
  //   FILE *fp = fopen("./inference_test_data/utils/bd_empty", "rb");
  //   auto blob = BinaryReadBlob<float>(fp);
  //   EXPECT_EQ(blob->count(), 0);
  //   fclose(fp);
  // }

  // // write and read non-empty blob
  // {
  //   base::Blob<float> blob;
  //   blob.Reshape({1, 2});
  //   blob.mutable_cpu_data()[0] = 0;
  //   blob.mutable_cpu_data()[1] = 1;
  //   FILE *fp = fopen("./inference_test_data/utils/bd_non_empty", "wb");
  //   BinaryWriteBlob(fp, blob);
  //   fclose(fp);
  // }
  // {
  //   char name[kMaxStrLen];
  //   FILE *fp = fopen("./inference_test_data/utils/bd_non_empty", "rb");
  //   auto blob = BinaryReadBlob<float>(fp);
  //   EXPECT_EQ(blob->count(), 2);
  //   EXPECT_EQ(blob->cpu_data()[0], 0);
  //   EXPECT_EQ(blob->cpu_data()[1], 1);
  //   fclose(fp);
  // }
}

TEST(BinaryDataTest, test_dict_io) {
  // // write to non-exist directory
  // {
  //   std::map<std::string, boost::shared_ptr<base::Blob<float>>> data_dict;
  //   EXPECT_FALSE(
  //       BinaryWriteFile("./inference_test_data/utils/none/bd_dict",
  //       data_dict));
  // }

  // // write empty dict
  // {
  //   std::map<std::string, boost::shared_ptr<base::Blob<float>>> data_dict;
  //   EXPECT_TRUE(BinaryWriteFile("./inference_test_data/utils/bd_dict_empty",
  //                               data_dict));
  // }

  // // write non-empty dict
  // {
  //   std::map<std::string, boost::shared_ptr<base::Blob<float>>> data_dict;
  //   boost::shared_ptr<base::Blob<float>> blob(new base::Blob<float>());
  //   blob->Reshape({1, 2});
  //   data_dict["b2d"] = blob;
  //   EXPECT_TRUE(
  //       BinaryWriteFile("./inference_test_data/utils/bd_dict", data_dict));
  // }

  // // read non-exist dict
  // {
  //   auto data_dict =
  //       BinaryReadFile<float>("./inference_test_data/utils/none/bd_dict");
  // }

  // // read empty dict
  // {
  //   auto data_dict =
  //       BinaryReadFile<float>("./inference_test_data/utils/bd_dict_empty");
  // }

  // // read non-empty dict
  // {
  //   auto data_dict =
  //       BinaryReadFile<float>("./inference_test_data/utils/bd_dict");
  // }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
