
#ifndef CYBERTRON_TF2_BUFFER_CORE_H_
#define CYBERTRON_TF2_BUFFER_CORE_H_

#include "transform_storage.h"

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cybertron/base/signal.h"
#include "cybertron/proto/common_geometry.pb.h"
#include "cybertron/time/duration.h"
#include "cybertron/time/time.h"

//////////////////////////backwards startup for porting
//#include "tf/tf.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

typedef std::pair<cybertron::Time, CompactFrameID> P_TimeAndFrameID;
typedef uint32_t TransformableCallbackHandle;
typedef uint64_t TransformableRequestHandle;

class TimeCacheInterface;
typedef std::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

enum TransformableResult {
  TransformAvailable,
  TransformFailure,
};

/** \brief A Class which provides coordinate transforms between any two frames
 *in a system.
 *
 * This class provides a simple interface to allow recording and lookup of
 * relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms which
 *define the relationship between all coordinate frames.
 * For example your typical robot would have a transform from global to real
 *world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to
 *hand.
 * libTF is designed to take care of all the intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for generating the
 *transform into that frame from it's parent and a reference to the parent
 *frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the exception
 *tf::LookupException
 */
class BufferCore {
 public:
  /************* Constants ***********************/
  static double DEFAULT_CACHE_TIME;
  static uint32_t MAX_GRAPH_DEPTH;

  /** Constructor
   * \param interpolating Whether to interpolate, if this is false the closest
   *value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  BufferCore();
  void setCacheTime(cybertron::Duration cache_time);
  // BufferCore(cybertron::Duration cache_time_ =
  //               cybertron::Duration(DEFAULT_CACHE_TIME));
  virtual ~BufferCore(void);

  /** \brief Clear all data */
  void clear();

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * \param is_static Record this transform as a static transform.  It will be
   * good accybertrons all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  bool setTransform(const adu::common::TransformStamped& transform,
                    const std::string& authority, bool is_static = false);

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0
   *will get the latest)
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  adu::common::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const cybertron::Time& time) const;

  /** \brief Get the transform between two frames by frame ID assuming fixed
   *frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0
   *will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated.
   *(0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant
   *in time.
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */

  adu::common::TransformStamped lookupTransform(
      const std::string& target_frame, const cybertron::Time& target_time,
      const std::string& source_frame, const cybertron::Time& source_time,
      const std::string& fixed_frame) const;

  /** \brief Lookup the twist of the tracking_frame with respect to the
   *observation frame in the reference_frame using the reference point
   * \param tracking_frame The frame to track
   * \param observation_frame The frame from which to measure the twist
   * \param reference_frame The reference frame in which to express the twist
   * \param reference_point The reference point with which to express the twist
   * \param reference_point_frame The frame_id in which the reference point is
   *expressed
   * \param time The time at which to get the velocity
   * \param duration The period over which to average
   * \return twist The twist output
   *
   * This will compute the average velocity on the interval
   * (time - duration/2, time+duration/2). If that is too close to the most
   * recent reading, in which case it will shift the interval up to
   * duration/2 to prevent extrapolation.
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   *
   * New in geometry 1.1
   */
  /*
  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string&
  observation_frame, const std::string& reference_frame,
                const tf::Point & reference_point, const std::string&
  reference_point_frame,
                const cybertron::Time& time, const cybertron::Time&
  averaging_interval) const;
  */
  /** \brief lookup the twist of the tracking frame with respect to the
   *observational frame
   *
   * This is a simplified version of
   * lookupTwist with it assumed that the reference point is the
   * origin of the tracking frame, and the reference frame is the
   * observation frame.
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   *
   * New in geometry 1.1
   */
  /*
  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string&
  observation_frame,
                const cybertron::Time& time, const cybertron::Time&
  averaging_interval) const;
  */
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the
   * transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const cybertron::Time& time,
                    std::string* error_msg = NULL) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in
   * time
   * \param error_msg A pointer to a string which will be filled with why the
   * transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  bool canTransform(const std::string& target_frame,
                    const cybertron::Time& target_time,
                    const std::string& source_frame,
                    const cybertron::Time& source_time,
                    const std::string& fixed_frame,
                    std::string* error_msg = NULL) const;

  /** \brief A way to see what frames have been cached in yaml format
   * Useful for debugging tools
   */
  std::string allFramesAsYAML(double current_time) const;

  /** Backwards compatibility for #84
  */
  std::string allFramesAsYAML() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsString() const;

  typedef std::function<void(TransformableRequestHandle request_handle,
                             const std::string& target_frame,
                             const std::string& source_frame,
                             cybertron::Time time, TransformableResult result)>
      TransformableCallback;

  /// \brief Internal use only
  TransformableCallbackHandle addTransformableCallback(
      const TransformableCallback& cb);
  /// \brief Internal use only
  void removeTransformableCallback(TransformableCallbackHandle handle);
  /// \brief Internal use only
  TransformableRequestHandle addTransformableRequest(
      TransformableCallbackHandle handle, const std::string& target_frame,
      const std::string& source_frame, cybertron::Time time);
  /// \brief Internal use only
  void cancelTransformableRequest(TransformableRequestHandle handle);

  // Tell the buffer that there are multiple threads serviciing it.
  // This is useful for derived classes to know if they can block or not.
  void setUsingDedicatedThread(bool value) { using_dedicated_thread_ = value; };
  // Get the state of using_dedicated_thread_
  bool isUsingDedicatedThread() const { return using_dedicated_thread_; };

  /* Backwards compatability section for tf::Transformer you should not use
   * these
   */

  /**
   * \brief Add a callback that happens when a new transform has arrived
   *
   * \param callback The callback, of the form void func();
   * \return A cybertron::base::Connection object that can be used to remove
   *this
   * listener
   */
  cybertron::base::Connection<> _addTransformsChangedListener(
      std::function<void(void)> callback);
  void _removeTransformsChangedListener(cybertron::base::Connection<> c);

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  bool _frameExists(const std::string& frame_id_str) const;

  /**@brief Fill the parent of a frame.
   * @param frame_id The frame id of the frame in question
   * @param parent The reference to the string to fill the parent
   * Returns true unless "NO_PARENT" */
  bool _getParent(const std::string& frame_id, cybertron::Time time,
                  std::string& parent) const;

  /** \brief A way to get a std::vector of available frame ids */
  void _getFrameStrings(std::vector<std::string>& ids) const;

  CompactFrameID _lookupFrameNumber(const std::string& frameid_str) const {
    return lookupFrameNumber(frameid_str);
  }
  CompactFrameID _lookupOrInsertFrameNumber(const std::string& frameid_str) {
    return lookupOrInsertFrameNumber(frameid_str);
  }

  int _getLatestCommonTime(CompactFrameID target_frame,
                           CompactFrameID source_frame, cybertron::Time& time,
                           std::string* error_string) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return getLatestCommonTime(target_frame, source_frame, time, error_string);
  }

  CompactFrameID _validateFrameId(const char* function_name_arg,
                                  const std::string& frame_id) const {
    return validateFrameId(function_name_arg, frame_id);
  }

  /**@brief Get the duration over which this transformer will cache */
  cybertron::Duration getCacheLength() { return cache_time_; }

  /** \brief Backwards compatabilityA way to see what frames have been cached
   * Useful for debugging
   */
  std::string _allFramesAsDot(double current_time) const;
  std::string _allFramesAsDot() const;

  /** \brief Backwards compatabilityA way to see what frames are in a chain
   * Useful for debugging
   */
  void _chainAsVector(const std::string& target_frame,
                      cybertron::Time target_time,
                      const std::string& source_frame,
                      cybertron::Time source_time,
                      const std::string& fixed_frame,
                      std::vector<std::string>& output) const;

 private:
  /** \brief A way to see what frames have been cached
   * Useful for debugging. Use this call internally.
   */
  std::string allFramesAsStringNoLock() const;

  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first
   * time. */
  typedef std::vector<TimeCacheInterfacePtr> V_TimeCacheInterface;
  V_TimeCacheInterface frames_;

  /** \brief A mutex to protect testing and allocating new frames on the above
   * vector. */
  mutable std::mutex frame_mutex_;

  /** \brief A map from string frame ids to CompactFrameID */
  typedef std::unordered_map<std::string, CompactFrameID>
      M_StringToCompactFrameID;
  M_StringToCompactFrameID frameIDs_;
  /** \brief A map from CompactFrameID frame_id_numbers to string for debugging
   * and output */
  std::vector<std::string> frameIDs_reverse;
  /** \brief A map to lookup the most recent authority for a given frame */
  std::map<CompactFrameID, std::string> frame_authority_;

  /// How long to cache transform history
  cybertron::Duration cache_time_ = cybertron::Duration(DEFAULT_CACHE_TIME);

  typedef std::unordered_map<TransformableCallbackHandle, TransformableCallback>
      M_TransformableCallback;
  M_TransformableCallback transformable_callbacks_;
  uint32_t transformable_callbacks_counter_;
  std::mutex transformable_callbacks_mutex_;

  struct TransformableRequest {
    cybertron::Time time;
    TransformableRequestHandle request_handle;
    TransformableCallbackHandle cb_handle;
    CompactFrameID target_id;
    CompactFrameID source_id;
    std::string target_string;
    std::string source_string;
  };
  typedef std::vector<TransformableRequest> V_TransformableRequest;
  V_TransformableRequest transformable_requests_;
  std::mutex transformable_requests_mutex_;
  uint64_t transformable_requests_counter_;

  struct RemoveRequestByCallback;
  struct RemoveRequestByID;

  // Backwards compatability for tf message_filter
  typedef cybertron::base::Signal<> TransformsChangedSignal;
  /// Signal which is fired whenever new transform data has arrived, from the
  /// thread the data arrived in
  TransformsChangedSignal _transforms_changed_;

  /************************* Internal Functions ****************************/

  /** \brief An accessor to get a frame, which will throw an exception if the
   *frame is no there.
   * \param frame_number The frameID of the desired Reference Frame
   *
   * This is an internal function which will get the pointer to the frame
   *associated with the frame id
   * Possible Exception: tf::LookupException
   */
  TimeCacheInterfacePtr getFrame(CompactFrameID c_frame_id) const;

  TimeCacheInterfacePtr allocateFrame(CompactFrameID cfid, bool is_static);

  bool warnFrameId(const char* function_name_arg,
                   const std::string& frame_id) const;
  CompactFrameID validateFrameId(const char* function_name_arg,
                                 const std::string& frame_id) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupFrameNumber(const std::string& frameid_str) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupOrInsertFrameNumber(const std::string& frameid_str);

  /// Number to string frame lookup may throw LookupException if number invalid
  const std::string& lookupFrameString(CompactFrameID frame_id_num) const;

  void createConnectivityErrorString(CompactFrameID source_frame,
                                     CompactFrameID target_frame,
                                     std::string* out) const;

  /**@brief Return the latest cybertrontime which is common accybertrons the
   * spanning set
   * zero if fails to ccybertrons */
  int getLatestCommonTime(CompactFrameID target_frame,
                          CompactFrameID source_frame, cybertron::Time& time,
                          std::string* error_string) const;

  template <typename F>
  int walkToTopParent(F& f, cybertron::Time time, CompactFrameID target_id,
                      CompactFrameID source_id,
                      std::string* error_string) const;

  /**@brief Traverse the transform tree. If frame_chain is not NULL, store the
   * traversed frame tree in vector frame_chain.
   * */
  template <typename F>
  int walkToTopParent(F& f, cybertron::Time time, CompactFrameID target_id,
                      CompactFrameID source_id, std::string* error_string,
                      std::vector<CompactFrameID>* frame_chain) const;

  void testTransformableRequests();
  bool canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                            const cybertron::Time& time,
                            std::string* error_msg) const;
  bool canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                          const cybertron::Time& time,
                          std::string* error_msg) const;

  // Whether it is safe to use canTransform with a timeout. (If another thread
  // is not provided it will always timeout.)
  bool using_dedicated_thread_;
};
}
}
}

#endif  // CYBERTRON_TF2_BUFFER_CORE_H_
