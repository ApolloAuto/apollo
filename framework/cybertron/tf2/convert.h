
#ifndef CYBERTRON_TF2_CONVERT_H_
#define CYBERTRON_TF2_CONVERT_H_

#include <cybertron/tf2/transform_datatypes.h>
#include <cybertron/tf2/exceptions.h>
#include <cybertron/tf2/impl/convert.h>

#include "cybertron/time/time.h"
#include "cybertron/proto/common_geometry.pb.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any
 *given datatype.
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data
 *in and the method should be mutation safe.
 * \param transform The transform to apply to data_in to fill data_out.
 *
 * This method needs to be implemented by client library developers
 */
template <typename T>
void doTransform(const T& data_in, T& data_out,
                 const adu::common::TransformStamped& transform);

/**\brief Get the timestamp from data
 * \param t The data input.
 * \return The timestamp associated with the data.
 */
template <typename T>
const cybertron::Time& getTimestamp(const T& t);

/**\brief Get the frame_id from data
 * \param t The data input.
 * \return The frame_id associated with the data.
 */
template <typename T>
const std::string& getFrameId(const T& t);

/* An implementation for Stamped<P> datatypes */
template <typename P>
const cybertron::Time& getTimestamp(const tf2::Stamped<P>& t) {
  return t.stamp_;
}

/* An implementation for Stamped<P> datatypes */
template <typename P>
const std::string& getFrameId(const tf2::Stamped<P>& t) {
  return t.frame_id_;
}

/** Function that converts from one type to a ROS message type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is
 * used in the "convert" function.
 * \param a an object of whatever type
 * \return the conversion as a ROS message
 */
template <typename A, typename B>
B toMsg(const A& a);

/** Function that converts from a ROS message type to another type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is used
 * in the "convert" function.
 * \param a a ROS message to convert from
 * \param b the object to convert to
 */
template <typename A, typename B>
void fromMsg(const A&, B& b);

/** Function that converts any type to any type (messages or not).
 * Matching toMsg and from Msg conversion functions need to exist.
 * If they don't exist or do not apply (for example, if your two
 * classes are ROS messages), just write a specialization of the function.
 * \param a an object to convert from
 * \param b the object to convert to
 */
template <typename A, typename B>
void convert(const A& a, B& b) {
  // printf("In double type convert\n");
  // impl::Converter<cybertron::message_traits::IsMessage<A>::value,
  // cybertron::message_traits::IsMessage<B>::value>::convert(a, b);
}

template <typename A>
void convert(const A& a1, A& a2) {
  // printf("In single type convert\n");
  if (&a1 != &a2) a2 = a1;
}
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_CONVERT_H_
