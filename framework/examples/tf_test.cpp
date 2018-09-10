
#include "cybertron/cybertron.h"
#include "cybertron/tf2_cybertron/static_transform_broadcaster.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"
#include "cybertron/tf2_cybertron/transform_listener.h"

#include "cybertron/proto/common_geometry.pb.h"

void test() {
  auto buffer = apollo::cybertron::tf2_cybertron::Buffer::Instance();
  ;

  apollo::cybertron::tf2_cybertron::TransformBroadcaster br;
  adu::common::TransformStamped test;
  test.mutable_header()->set_stamp(1498034408514957926);
  test.mutable_header()->set_frame_id("world");
  // test.mutable_cyber_header()->set_meta_stamp(1234);
  test.set_child_frame_id("car");

  test.mutable_transform()->mutable_translation()->set_x(1.12312133);
  test.mutable_transform()->mutable_translation()->set_y(2.12312133);
  test.mutable_transform()->mutable_translation()->set_z(3.12312133);
  test.mutable_transform()->mutable_rotation()->set_qx(0);
  test.mutable_transform()->mutable_rotation()->set_qy(0);
  test.mutable_transform()->mutable_rotation()->set_qz(0);
  test.mutable_transform()->mutable_rotation()->set_qw(1);

  AINFO << "wo shi 111";

  br.sendTransform(test);

  adu::common::TransformStamped test_new;
  test_new.mutable_header()->set_stamp(1498034408514958926);
  test_new.mutable_header()->set_frame_id("world");
  test_new.set_child_frame_id("car");

  test_new.mutable_transform()->mutable_translation()->set_x(1.12312133);
  test_new.mutable_transform()->mutable_translation()->set_y(2.12312133);
  test_new.mutable_transform()->mutable_translation()->set_z(3.12312133);
  test_new.mutable_transform()->mutable_rotation()->set_qx(0);
  test_new.mutable_transform()->mutable_rotation()->set_qy(0);
  test_new.mutable_transform()->mutable_rotation()->set_qz(0);
  test_new.mutable_transform()->mutable_rotation()->set_qw(1);

  br.sendTransform(test_new);
  std::vector<adu::common::TransformStamped> test_new_vec;
  test_new_vec.push_back(test_new);
  br.sendTransform(test_new_vec);
  std::shared_ptr<apollo::cybertron::Node> node(
      apollo::cybertron::CreateNode("tftt"));
  apollo::cybertron::tf2_cybertron::StaticTransformBroadcaster broadcaster(
      node);

  adu::common::TransformStamped msg;
  msg.mutable_transform()->mutable_translation()->set_x(1);
  msg.mutable_transform()->mutable_translation()->set_y(1);
  msg.mutable_transform()->mutable_translation()->set_z(1);
  msg.mutable_transform()->mutable_rotation()->set_qx(0);
  msg.mutable_transform()->mutable_rotation()->set_qy(0);
  msg.mutable_transform()->mutable_rotation()->set_qz(0);
  msg.mutable_transform()->mutable_rotation()->set_qw(1);
  msg.mutable_header()->set_stamp(
      apollo::cybertron::Time::Now().ToNanosecond());
  msg.mutable_header()->set_frame_id("car");
  msg.set_child_frame_id("car1");
  broadcaster.sendTransform(msg);

  AINFO << "wo shi 111";
  sleep(1);
  
  std::string err_string;
  bool res1 = buffer->canTransform(
      "world", "car", apollo::cybertron::Time((uint64_t)0), 0.1,
      &err_string);
  bool res2 = buffer->canTransform(
      "world", "car1", 0);
  AINFO << "wo shi 111";
  if (res1) AINFO << "Transform:" << res1;
  AINFO << "wo shi 111";
  if (res2) AINFO << "Transform:" << res2;
}

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  test();
  return 0;
}
