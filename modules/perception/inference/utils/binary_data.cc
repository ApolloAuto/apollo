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

#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/inference/utils/binary_data.h"

namespace apollo {
namespace perception {
namespace inference {

inline std::string get_dtype(const base::Blob<double> &blob) {
  return "float64";
}

inline std::string get_dtype(const base::Blob<float> &blob) {
  return "float32";
}

size_t BinaryReadString(FILE *fp, char *name) {
  size_t len = 0;
  size_t nmemb = fread(&len, sizeof(len), 1, fp);
  if (nmemb != 1 || len == 0) {
    return 0;
  }
  CHECK_LT(len, kMaxStrLen);
  nmemb = fread(name, sizeof(name[0]), len, fp);
  CHECK_EQ(nmemb, len);
  name[len] = 0;
  return len;
}

size_t BinaryWriteString(FILE *fp, const std::string &str) {
  size_t len = str.length();
  fwrite(&len, sizeof(len), 1, fp);
  fwrite(str.c_str(), sizeof(str[0]), len, fp);
  return len;
}

template <typename Dtype>
boost::shared_ptr<base::Blob<Dtype>> BinaryReadBlob(FILE *fp) {
  int ndim;
  boost::shared_ptr<base::Blob<Dtype>> blob(new base::Blob<Dtype>());
  char dtype[kMaxStrLen];

  // read dtype
  size_t nmemb = BinaryReadString(fp, dtype);
  CHECK_GT(nmemb, 0);
  CHECK_EQ(get_dtype(*blob), dtype);

  // read dims
  nmemb = fread(&ndim, sizeof(ndim), 1, fp);
  CHECK_EQ(nmemb, 1);
  std::vector<int> shape(ndim);
  for (int i = 0; i < ndim; ++i) {
    nmemb = fread(&shape[i], sizeof(shape[i]), 1, fp);
    CHECK_EQ(nmemb, 1);
  }
  if (ndim == 0) {
    return blob;
  }

  // init blob
  blob->Reshape(shape);
  CHECK_GE(blob->count(), kMinDim);

  // read data
  nmemb = fread(blob->mutable_cpu_data(), sizeof(Dtype), blob->count(), fp);
  CHECK_EQ(nmemb, blob->count());

  return blob;
}

template <typename Dtype>
void BinaryWriteBlob(FILE *fp, const base::Blob<Dtype> &blob) {
  int ndim, dim;
  // write dtype
  BinaryWriteString(fp, get_dtype(blob));
  // write dims
  ndim = blob.num_axes();
  fwrite(&ndim, sizeof(ndim), 1, fp);
  for (int i = 0; i < ndim; ++i) {
    dim = blob.shape(i);
    fwrite(&dim, sizeof(dim), 1, fp);
  }

  // write data
  if (blob.count() > 0) {
    fwrite(blob.cpu_data(), sizeof(Dtype), blob.count(), fp);
  }
}

template boost::shared_ptr<base::Blob<float>> BinaryReadBlob(FILE *fp);
template boost::shared_ptr<base::Blob<double>> BinaryReadBlob(FILE *fp);

template void BinaryWriteBlob(FILE *fp, const base::Blob<float> &blob);
template void BinaryWriteBlob(FILE *fp, const base::Blob<double> &blob);

template <typename Dtype>
std::map<std::string, boost::shared_ptr<base::Blob<Dtype>>> BinaryReadFile(
    const char *file_path) {
  char name[kMaxStrLen];
  std::map<std::string, boost::shared_ptr<base::Blob<Dtype>>> data_dict;

  FILE *fp = fopen(file_path, "rb");
  if (NULL == fp) {
    AERROR << "Failed opening Binaryary file: " << file_path;
    return data_dict;
  }

  // read blob name
  while (BinaryReadString(fp, name)) {
    // insert into data_dict
    data_dict[name] = BinaryReadBlob<Dtype>(fp);
  }

  fclose(fp);
  return data_dict;
}

template <typename Btype>
bool BinaryWriteFile(const char *file_path,
                     const std::map<std::string, Btype> &data_dict) {
  FILE *fp = fopen(file_path, "wb");
  if (NULL == fp) {
    AERROR << "Failed opening Binaryary file: " << file_path;
    return false;
  }

  typename std::map<std::string, Btype>::const_iterator it = data_dict.begin();
  while (it != data_dict.end()) {
    // write blob name
    BinaryWriteString(fp, it->first);
    // write blob
    BinaryWriteBlob(fp, *it->second);

    ++it;
  }
  int end = 0;
  fwrite(&end, sizeof(end), 1, fp);

  fclose(fp);
  return true;
}

template std::map<std::string, boost::shared_ptr<base::Blob<float>>>
BinaryReadFile(const char *file_path);
template std::map<std::string, boost::shared_ptr<base::Blob<double>>>
BinaryReadFile(const char *file_path);

template bool BinaryWriteFile(
    const char *file_path,
    const std::map<std::string, boost::shared_ptr<base::Blob<float>>>
        &data_dict);
template bool BinaryWriteFile(
    const char *file_path,
    const std::map<std::string, boost::shared_ptr<base::Blob<double>>>
        &data_dict);

template bool BinaryWriteFile(
    const char *file_path,
    const std::map<std::string, base::Blob<float> *> &data_dict);
template bool BinaryWriteFile(
    const char *file_path,
    const std::map<std::string, base::Blob<double> *> &data_dict);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
