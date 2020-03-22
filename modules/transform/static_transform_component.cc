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

#include "modules/transform/static_transform_component.h"

#include "yaml-cpp/yaml.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace transform {

bool StaticTransformComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "Parse conf file failed, " << ConfigFilePath();
    return false;
  }
  cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_tf_static_topic);
  attr.mutable_qos_profile()->CopyFrom(
      cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  writer_ = node_->CreateWriter<TransformStampeds>(attr);
  SendTransforms();
  return true;
}

void StaticTransformComponent::SendTransforms() {
  std::vector<TransformStamped> tranform_stamped_vec;
  for (auto& extrinsic_file : conf_.extrinsic_file()) {
    if (extrinsic_file.enable()) {
      AINFO << "Broadcast static transform, frame id ["
            << extrinsic_file.frame_id() << "], child frame id ["
            << extrinsic_file.child_frame_id() << "]";
      TransformStamped transform;
      if (ParseFromYaml(extrinsic_file.file_path(), &transform)) {
        tranform_stamped_vec.emplace_back(transform);
      }
    }
  }
  SendTransform(tranform_stamped_vec);
}

bool StaticTransformComponent::ParseFromYaml(
    const std::string& file_path, TransformStamped* transform_stamped) {
  if (!cyber::common::PathExists(file_path)) {
    AERROR << "Extrinsic yaml file does not exist: " << file_path;
    return false;
  }
  YAML::Node tf = YAML::LoadFile(file_path);
  try {
    transform_stamped->mutable_header()->set_frame_id(
        tf["header"]["frame_id"].as<std::string>());
    transform_stamped->set_child_frame_id(
        tf["child_frame_id"].as<std::string>());
    // translation
    auto translation =
        transform_stamped->mutable_transform()->mutable_translation();
    translation->set_x(tf["transform"]["translation"]["x"].as<double>());
    translation->set_y(tf["transform"]["translation"]["y"].as<double>());
    translation->set_z(tf["transform"]["translation"]["z"].as<double>());
    // rotation
    auto rotation = transform_stamped->mutable_transform()->mutable_rotation();
    rotation->set_qx(tf["transform"]["rotation"]["x"].as<double>());
    rotation->set_qy(tf["transform"]["rotation"]["y"].as<double>());
    rotation->set_qz(tf["transform"]["rotation"]["z"].as<double>());
    rotation->set_qw(tf["transform"]["rotation"]["w"].as<double>());
  } catch (...) {
    AERROR << "Extrinsic yaml file parse failed: " << file_path;
    return false;
  }
  return true;
}

void StaticTransformComponent::SendTransform(
    const std::vector<TransformStamped>& msgtf) {
  for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in) {
    bool match_found = false;
    for (auto& it_msg : *transform_stampeds_.mutable_transforms()) {
      if (it_in->child_frame_id() == it_msg.child_frame_id()) {
        it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (!match_found) {
      *transform_stampeds_.add_transforms() = *it_in;
    }
  }

  common::util::FillHeader(node_->Name(), &transform_stampeds_);
  writer_->Write(transform_stampeds_);
}

}  // namespace transform
}  // namespace apollo
