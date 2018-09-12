#include "cybertron/tf2_cybertron/buffer.h"
#include <sstream>
#include "cybertron/cybertron.h"
#include "cybertron/proto/common_geometry.pb.h"

namespace apollo {
namespace cybertron {
namespace tf2_cybertron {

Buffer::Buffer() : BufferCore() { Init(); }

int Buffer::Init() {
  std::string node_name = "transform_listener_" +
                          std::to_string(cybertron::Time::Now().ToNanosecond());
  node_ = cybertron::CreateNode(node_name);
  proto::RoleAttributes attr;
  attr.set_channel_name("/tf");
  message_subscriber_tf_ = node_->CreateReader<adu::common::TransformStampeds>(
      attr, [&](const std::shared_ptr<const adu::common::TransformStampeds>&
                    msg_evt) { SubscriptionCallbackImpl(msg_evt, false); });

  apollo::cybertron::proto::RoleAttributes attr_static;
  attr_static.set_channel_name("/tf_static");
  attr_static.mutable_qos_profile()->CopyFrom(
      transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  message_subscriber_tf_static_ =
      node_->CreateReader<adu::common::TransformStampeds>(
          attr_static,
          [&](const std::shared_ptr<adu::common::TransformStampeds>& msg_evt) {
            SubscriptionCallbackImpl(msg_evt, true);
          });

  return cybertron::SUCC;
}

void Buffer::SubscriptionCallback(
    const std::shared_ptr<const adu::common::TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, false);
}

void Buffer::StaticSubscriptionCallback(
    const std::shared_ptr<const adu::common::TransformStampeds>& msg_evt) {
  SubscriptionCallbackImpl(msg_evt, true);
}

void Buffer::SubscriptionCallbackImpl(
    const std::shared_ptr<const adu::common::TransformStampeds>& msg_evt,
    bool is_static) {
  cybertron::Time now = cybertron::Time::Now();
  std::string authority =
      "cybertron_tf";  // msg_evt.getPublisherName(); // lookup the authority
  if (now.ToNanosecond() < last_update_.ToNanosecond()) {
    AINFO << "Detected jump back in time. Clearing TF buffer.";
    clear();
    //cache static transform stamped again.
    for (auto &msg : static_msgs_) {
      setTransform(msg, authority, true);
    }
  }
  last_update_ = now;

  for (int i = 0; i < msg_evt->transforms_size(); i++) {
    try {
      geometry_msgs::TransformStamped trans_stamped;

      //header
      trans_stamped.header.stamp = msg_evt->transforms(i).header().stamp();
      trans_stamped.header.frame_id = msg_evt->transforms(i).header().frame_id();
      trans_stamped.header.seq = msg_evt->transforms(i).header().sequence_num();

      //child_frame_id
      trans_stamped.child_frame_id = msg_evt->transforms(i).child_frame_id();

      //translation
      trans_stamped.transform.translation.x =
          msg_evt->transforms(i).transform().translation().x();
      trans_stamped.transform.translation.y =
          msg_evt->transforms(i).transform().translation().y();
      trans_stamped.transform.translation.z =
          msg_evt->transforms(i).transform().translation().z();

      //rotation
      trans_stamped.transform.rotation.x =
          msg_evt->transforms(i).transform().rotation().qx();
      trans_stamped.transform.rotation.y =
          msg_evt->transforms(i).transform().rotation().qy();
      trans_stamped.transform.rotation.z =
          msg_evt->transforms(i).transform().rotation().qz();
      trans_stamped.transform.rotation.w =
          msg_evt->transforms(i).transform().rotation().qw();

      if (is_static){
        static_msgs_.push_back(trans_stamped);
      }
      setTransform(trans_stamped, authority, is_static);
    }

    catch (tf2::TransformException& ex) {
      std::string temp = ex.what();
      AERROR << "Failure to set recieved transform:" << temp.c_str();
    }
  }
};

void Buffer::tf2MsgToCyber(
    const geometry_msgs::TransformStamped& tf2_trans_stamped,
    adu::common::TransformStamped& trans_stamped) const {
  // header
  trans_stamped.mutable_header()->set_stamp(tf2_trans_stamped.header.stamp);
  trans_stamped.mutable_header()->set_frame_id(
      tf2_trans_stamped.header.frame_id);

  // child_frame_id
  trans_stamped.set_child_frame_id(tf2_trans_stamped.child_frame_id);

  // translation
  trans_stamped.mutable_transform()->mutable_translation()->set_x(
      tf2_trans_stamped.transform.translation.x);
  trans_stamped.mutable_transform()->mutable_translation()->set_y(
      tf2_trans_stamped.transform.translation.y);
  trans_stamped.mutable_transform()->mutable_translation()->set_z(
      tf2_trans_stamped.transform.translation.z);

  // rotation
  trans_stamped.mutable_transform()->mutable_rotation()->set_qx(
      tf2_trans_stamped.transform.rotation.x);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qy(
      tf2_trans_stamped.transform.rotation.y);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qz(
      tf2_trans_stamped.transform.rotation.z);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qw(
      tf2_trans_stamped.transform.rotation.w);
}

adu::common::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const cybertron::Time& time, const float timeout_second) const {
  (void)timeout_second;
  tf2::Time tf2_time(time.ToNanosecond());
  geometry_msgs::TransformStamped tf2_trans_stamped =
      lookupTransform(target_frame, source_frame, tf2_time);
  adu::common::TransformStamped trans_stamped;
  tf2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

adu::common::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const cybertron::Time& target_time,
    const std::string& source_frame, const cybertron::Time& source_time,
    const std::string& fixed_frame, const float timeout_second) const {

  (void)timeout_second;
  geometry_msgs::TransformStamped tf2_trans_stamped =
      lookupTransform(target_frame, target_time.ToNanosecond(), source_frame,
                      source_time.ToNanosecond(), fixed_frame);
  adu::common::TransformStamped trans_stamped;
  tf2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const cybertron::Time& time,
                          const float timeout_second,
                          std::string* errstr) const {
  uint64_t timeout_ns = timeout_second * 1000000000;
  uint64_t start_time = cybertron::Time::Now().ToNanosecond();
  while (cybertron::Time::Now().ToNanosecond() < start_time + timeout_ns &&
         !canTransform(target_frame, source_frame, time.ToNanosecond(), errstr) &&
         !cybertron::IsShutdown())  // Make sure we haven't been stopped (won't
                                    // work for
                                    // pytf)
  {
    usleep(3000);
  }
  bool retval = canTransform(target_frame, source_frame, time.ToNanosecond(), errstr);
  // conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const cybertron::Time& target_time,
                          const std::string& source_frame,
                          const cybertron::Time& source_time,
                          const std::string& fixed_frame,
                          const float timeout_second,
                          std::string* errstr) const {
  // if (!checkAndErrorDedicatedThreadPresent(errstr))
  //   return false;

  // poll for transform if timeout is set
  uint64_t timeout_ns = timeout_second * 1000000000;
  uint64_t start_time = cybertron::Time::Now().ToNanosecond();
  while (cybertron::Time::Now().ToNanosecond() < start_time + timeout_ns &&
         !canTransform(target_frame, target_time.ToNanosecond(), source_frame, source_time.ToNanosecond(),
                       fixed_frame) &&
         !cybertron::IsShutdown())  // Make sure we haven't been stopped
                                    // (won't work for pytf)
  {
    usleep(3000);
  }
  bool retval = canTransform(target_frame, target_time.ToNanosecond(), source_frame,
                             source_time.ToNanosecond(), fixed_frame, errstr);
  // conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}

}
}
}
