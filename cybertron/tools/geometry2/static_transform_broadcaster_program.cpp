
#include <cstdio>
#include <memory>

#include "cybertron/cybertron.h"
#include "cybertron/tf2_cybertron/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  std::shared_ptr<apollo::cybertron::Node> static_node(
      apollo::cybertron::CreateNode(
          "static_transform_publisher" +
          std::to_string(apollo::cybertron::Time::Now().ToNanosecond())));
  apollo::cybertron::tf2_cybertron::StaticTransformBroadcaster broadcaster(
      static_node);

  if (argc == 10) {
    if (strcmp(argv[8], argv[9]) == 0) {
      AERROR << "target_frame and source frame are the same, this cannot work:"
             << argv[8] << argv[9];
      return 1;
    }

    adu::common::TransformStamped msg;
    msg.mutable_transform()->mutable_translation()->set_x(atof(argv[1]));
    msg.mutable_transform()->mutable_translation()->set_y(atof(argv[2]));
    msg.mutable_transform()->mutable_translation()->set_z(atof(argv[3]));
    msg.mutable_transform()->mutable_rotation()->set_qx(atof(argv[4]));
    msg.mutable_transform()->mutable_rotation()->set_qy(atof(argv[5]));
    msg.mutable_transform()->mutable_rotation()->set_qz(atof(argv[6]));
    msg.mutable_transform()->mutable_rotation()->set_qw(atof(argv[7]));
    msg.mutable_header()->set_stamp(
        apollo::cybertron::Time::Now().ToNanosecond());
    msg.mutable_header()->set_frame_id(argv[8]);
    msg.set_child_frame_id(argv[9]);

    broadcaster.sendTransform(msg);
    while (apollo::cybertron::OK()) {
      sleep(1);
    }

    return 0;
  } else if (argc == 9) {
    if (strcmp(argv[7], argv[8]) == 0) {
      AERROR << "target_frame and source frame are the same, this cannot work:"
             << argv[8] << argv[9];
      return 1;
    }

    adu::common::TransformStamped msg;
    msg.mutable_transform()->mutable_translation()->set_x(atof(argv[1]));
    msg.mutable_transform()->mutable_translation()->set_y(atof(argv[2]));
    msg.mutable_transform()->mutable_translation()->set_z(atof(argv[3]));

    tf2::Quaternion quat;
    quat.setRPY(atof(argv[6]), atof(argv[5]), atof(argv[4]));
    msg.mutable_transform()->mutable_rotation()->set_qx(quat.x());
    msg.mutable_transform()->mutable_rotation()->set_qy(quat.y());
    msg.mutable_transform()->mutable_rotation()->set_qz(quat.z());
    msg.mutable_transform()->mutable_rotation()->set_qw(quat.w());

    msg.mutable_header()->set_stamp(
        apollo::cybertron::Time::Now().ToNanosecond());
    msg.mutable_header()->set_frame_id(argv[7]);
    msg.set_child_frame_id(argv[8]);

    broadcaster.sendTransform(msg);
    while (apollo::cybertron::OK()) {
      sleep(1);
    }
    return 0;
  } else {
    printf("A command line utility for manually sending a transform.\n");
    printf(
        "Usage: static_transform_publisher x y z qx qy qz qw frame_id "
        "child_frame_id \n");
    printf("OR \n");
    printf(
        "Usage: static_transform_publisher x y z yaw pitch roll frame_id "
        "child_frame_id \n");
    printf(
        "\nThis transform is the transform of the coordinate frame from "
        "frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    AERROR << "static_transform_publisher exited due to not having the "
              "right number of arguments";
    return -1;
  }
}
