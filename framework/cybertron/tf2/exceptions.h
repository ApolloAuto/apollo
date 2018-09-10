
#ifndef CYBERTRON_TF2_EXCEPTIONS_H_
#define CYBERTRON_TF2_EXCEPTIONS_H_

#include <stdexcept>

namespace apollo {
namespace cybertron {
namespace tf2 {

/** \brief A base class for all tf2 exceptions
 * This inherits from cybertron::exception
 * which inherits from std::runtime_exception
 */
class TransformException : public std::runtime_error {
 public:
  TransformException(const std::string errorDescription)
      : std::runtime_error(errorDescription) {
    ;
  };
};

/** \brief An exception class to notify of no connection
 *
 * This is an exception class to be thrown in the case
 * that the Reference Frame tree is not connected between
 * the frames requested. */
class ConnectivityException : public TransformException {
 public:
  ConnectivityException(const std::string errorDescription)
      : tf2::TransformException(errorDescription) {
    ;
  };
};

/** \brief An exception class to notify of bad frame number
 *
 * This is an exception class to be thrown in the case that
 * a frame not in the graph has been attempted to be accessed.
 * The most common reason for this is that the frame is not
 * being published, or a parent frame was not set correctly
 * causing the tree to be broken.
 */
class LookupException : public TransformException {
 public:
  LookupException(const std::string errorDescription)
      : tf2::TransformException(errorDescription) {
    ;
  };
};

/** \brief An exception class to notify that the requested value would have
 *required extrapolation beyond current limits.
 *
 */
class ExtrapolationException : public TransformException {
 public:
  ExtrapolationException(const std::string errorDescription)
      : tf2::TransformException(errorDescription) {
    ;
  };
};

/** \brief An exception class to notify that one of the arguments is invalid
 *
 * usually it's an uninitalized Quaternion (0,0,0,0)
 *
 */
class InvalidArgumentException : public TransformException {
 public:
  InvalidArgumentException(const std::string errorDescription)
      : tf2::TransformException(errorDescription) {
    ;
  };
};

/** \brief An exception class to notify that a timeout has occured
 *
 *
 */
class TimeoutException : public TransformException {
 public:
  TimeoutException(const std::string errorDescription)
      : tf2::TransformException(errorDescription) {
    ;
  };
};
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_EXCEPTIONS_H_
