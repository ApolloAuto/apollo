#pragma once

#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "modules/perception/proto/rt.pb.h"

namespace apollo {
namespace perception {
namespace inference {
bool ReadProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message *proto);
bool ReadProtoFromBinaryFile(const std::string &filename,
                             google::protobuf::Message *proto);
bool loadNetParams(const std::string &param_file, NetParameter *param);
std::string locateFile(const std::string &path, const std::string &input);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
