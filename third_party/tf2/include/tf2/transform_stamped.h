#ifndef GEOMETRY_MSGS_TRANSFORM_STAMPED_H 
#define GEOMETRY_MSGS_TRANSFORM_STAMPED_H

#include <iostream>
#include <stdint.h>

namespace geometry_msgs {

struct Header {
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
  Header() : seq(0), stamp(0), frame_id("") {}
};

struct Vector3 {
  double x;
  double y;
  double z;
  Vector3() : x(0.0), y(0.0), z(0.0) {}
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
  Quaternion(): x(0.0), y(0.0), z(0.0), w(0.0) {}
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped { 
  Header header;
  std::string child_frame_id;
  Transform transform;
}; 

}

#endif
