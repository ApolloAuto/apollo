
#ifndef CYBERTRON_TF2_TRANSFORM_STORAGE_H_
#define CYBERTRON_TF2_TRANSFORM_STORAGE_H_

#include <cybertron/tf2/LinearMath/Vector3.h>
#include <cybertron/tf2/LinearMath/Quaternion.h>

#include "cybertron/time/time.h"
#include "cybertron/proto/common_geometry.pb.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

typedef uint32_t CompactFrameID;

/** \brief Storage for transforms and their parent */
class TransformStorage {
 public:
  TransformStorage();
  TransformStorage(const adu::common::TransformStamped& data,
                   CompactFrameID frame_id, CompactFrameID child_frame_id);

  TransformStorage(const TransformStorage& rhs) { *this = rhs; }

  TransformStorage& operator=(const TransformStorage& rhs) {
#if 01
    rotation_ = rhs.rotation_;
    translation_ = rhs.translation_;
    stamp_ = rhs.stamp_;
    frame_id_ = rhs.frame_id_;
    child_frame_id_ = rhs.child_frame_id_;
#endif
    return *this;
  }

  tf2::Quaternion rotation_;
  tf2::Vector3 translation_;
  cybertron::Time stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
};
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_TRANSFORM_STORAGE_H_
