
#ifndef CYBERTRON_TF2_TRANSFORM_DATATYPES_H_
#define CYBERTRON_TF2_TRANSFORM_DATATYPES_H_

#include <string>
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

/** \brief The data type which will be ccybertrons compatable with geometry_msgs
 * This is the tf2 datatype equivilant of a MessageStamped */
template <typename T>
class Stamped : public T {
 public:
  cybertron::Time stamp_;  ///< The timestamp associated with this data
  std::string frame_id_;   ///< The frame_id associated this data

  /** Default constructor */
  Stamped() : frame_id_("NO_ID_STAMPED_DEFAULT_CONSTRUCTION") {};  // Default
  // constructor used
  // only for
  // preallocation

  /** Full constructor */
  Stamped(const T& input, const cybertron::Time& timestamp,
          const std::string& frame_id)
      : T(input), stamp_(timestamp), frame_id_(frame_id) {};

  /** Copy Constructor */
  Stamped(const Stamped<T>& s)
      : T(s), stamp_(s.stamp_), frame_id_(s.frame_id_) {}

  /** Set the data element */
  void setData(const T& input) {
    *static_cast<T*>(this) = input;
  };
};

/** \brief Comparison Operator for Stamped datatypes */
template <typename T>
bool operator==(const Stamped<T>& a, const Stamped<T>& b) {
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
         static_cast<const T&>(a) == static_cast<const T&>(b);
};
}
}
}
#endif  // INCLUDE_CYBERTRON_TF2_TRANSFORM_DATATYPES_H_
