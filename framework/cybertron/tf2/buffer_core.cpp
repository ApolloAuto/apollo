
#include "cybertron/common/log.h"
#include "cybertron/tf2/buffer_core.h"
#include "cybertron/tf2/time_cache.h"
#include "cybertron/tf2/exceptions.h"

#include <assert.h>
#include <console_bridge/console.h>
#include "cybertron/tf2/LinearMath/Transform.h"

namespace apollo {
namespace cybertron {
namespace tf2 {
//!< The default amount of time to cache data in seconds
double BufferCore::DEFAULT_CACHE_TIME = 10.0;
//!< The default amount of time to cache data in seconds
uint32_t BufferCore::MAX_GRAPH_DEPTH = 1000UL;

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const adu::common::Transform& msg, tf2::Transform& tf2) {
  tf2 =
      tf2::Transform(tf2::Quaternion(msg.rotation().qx(), msg.rotation().qy(),
                                     msg.rotation().qz(), msg.rotation().qw()),
                     tf2::Vector3(msg.translation().x(), msg.translation().y(),
                                  msg.translation().z()));
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2, adu::common::Transform& msg) {
  msg.mutable_translation()->set_x(tf2.getOrigin().x());
  msg.mutable_translation()->set_y(tf2.getOrigin().y());
  msg.mutable_translation()->set_z(tf2.getOrigin().z());
  msg.mutable_rotation()->set_qx(tf2.getRotation().x());
  msg.mutable_rotation()->set_qy(tf2.getRotation().y());
  msg.mutable_rotation()->set_qz(tf2.getRotation().z());
  msg.mutable_rotation()->set_qw(tf2.getRotation().w());
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2,
                       ::adu::common::TransformStamped& msg,
                       cybertron::Time stamp, const std::string& frame_id,
                       const std::string& child_frame_id) {
  transformTF2ToMsg(tf2, *msg.mutable_transform());
  msg.mutable_header()->set_stamp(stamp.ToNanosecond());
  msg.mutable_header()->set_frame_id(frame_id);
  msg.set_child_frame_id(child_frame_id);
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos,
                       adu::common::Transform& msg) {
  msg.mutable_translation()->set_x(pos.x());
  msg.mutable_translation()->set_y(pos.y());
  msg.mutable_translation()->set_z(pos.z());
  msg.mutable_rotation()->set_qx(orient.x());
  msg.mutable_rotation()->set_qy(orient.y());
  msg.mutable_rotation()->set_qz(orient.z());
  msg.mutable_rotation()->set_qw(orient.w());
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos,
                       ::adu::common::TransformStamped& msg,
                       cybertron::Time stamp, const std::string& frame_id,
                       const std::string& child_frame_id) {
  transformTF2ToMsg(orient, pos, *msg.mutable_transform());
  msg.mutable_header()->set_stamp(stamp.ToNanosecond());
  msg.mutable_header()->set_frame_id(frame_id);
  msg.set_child_frame_id(child_frame_id);
}

void setIdentity(adu::common::Transform& tx) {
  tx.mutable_translation()->set_x(0);
  tx.mutable_translation()->set_y(0);
  tx.mutable_translation()->set_z(0);
  tx.mutable_rotation()->set_qx(0);
  tx.mutable_rotation()->set_qy(0);
  tx.mutable_rotation()->set_qz(0);
  tx.mutable_rotation()->set_qw(1);
}

bool startsWithSlash(const std::string& frame_id) {
  if (frame_id.size() > 0)
    if (frame_id[0] == '/') return true;
  return false;
}

std::string stripSlash(const std::string& in) {
  std::string out = in;
  if (startsWithSlash(in)) out.erase(0, 1);
  return out;
}

bool BufferCore::warnFrameId(const char* function_name_arg,
                             const std::string& frame_id) const {
  if (frame_id.size() == 0) {
    std::stringstream ss;
    ss << "Invalid argument passed to " << function_name_arg
       << " in tf2 frame_ids cannot be empty";
    AWARN << ss.str().c_str();
    return true;
  }

  if (startsWithSlash(frame_id)) {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "
       << function_name_arg
       << " in tf2 frame_ids cannot start with a '/' like: ";
    AWARN << ss.str().c_str();
    return true;
  }

  return false;
}

CompactFrameID BufferCore::validateFrameId(const char* function_name_arg,
                                           const std::string& frame_id) const {
  if (frame_id.empty()) {
    std::stringstream ss;
    ss << "Invalid argument passed to " << function_name_arg
       << " in tf2 frame_ids cannot be empty";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }

  if (startsWithSlash(frame_id)) {
    std::stringstream ss;
    ss << "Invalid argument \"" << frame_id << "\" passed to "
       << function_name_arg
       << " in tf2 frame_ids cannot start with a '/' like: ";
    throw tf2::InvalidArgumentException(ss.str().c_str());
  }

  CompactFrameID id = lookupFrameNumber(frame_id);
  if (id == 0) {
    std::stringstream ss;
    ss << "\"" << frame_id << "\" passed to " << function_name_arg
       << " does not exist. ";
    throw tf2::LookupException(ss.str().c_str());
  }

  return id;
}

BufferCore::BufferCore()
    : transformable_callbacks_counter_(0),
      transformable_requests_counter_(0),
      using_dedicated_thread_(false) {
  cache_time_ = cybertron::Duration(DEFAULT_CACHE_TIME);
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(TimeCacheInterfacePtr());
  frameIDs_reverse.push_back("NO_PARENT");
}

BufferCore::~BufferCore() {}

void BufferCore::clear() {
  // old_tf_.clear();

  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (frames_.size() > 1) {
    for (std::vector<TimeCacheInterfacePtr>::iterator cache_it =
             frames_.begin() + 1;
         cache_it != frames_.end(); ++cache_it) {
      if (*cache_it) (*cache_it)->clearList();
    }
  }
}

bool BufferCore::setTransform(const ::adu::common::TransformStamped& transform_in,
                              const std::string& authority, bool is_static) {
  /////BACKEARDS COMPATABILITY
  /* tf::StampedTransform tf_transform;
  tf::transformStampedMsgToTF(transform_in, tf_transform);
  if  (!old_tf_.setTransform(tf_transform, authority))
  {
    printf("Warning old setTransform Failed but was not caught\n");
    }*/

  /////// New implementation
  ::adu::common::TransformStamped stripped = transform_in;
  stripped.mutable_header()->set_frame_id(
      stripSlash(stripped.header().frame_id()));
  stripped.set_child_frame_id(stripSlash(stripped.child_frame_id()));

  bool error_exists = false;
  if (stripped.child_frame_id() == stripped.header().frame_id()) {
//    LOG_ERROR_FORMAT(
//        "%d%d TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with "
//        "frame_id and child_frame_id  \"%s\" because they are the same",
//        CYBERTRON_ERROR, TF_SELF_TRANSFORM_ERROR,
//        authority.c_str(), stripped.child_frame_id().c_str());
    error_exists = true;
  }

  if (stripped.child_frame_id() == "") {
//    LOG_ERROR_FORMAT(
//        "%d%d TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" "
//        "because child_frame_id not set ",
//        CYBERTRON_ERROR, TF_NO_CHILD_FRAME_ID_ERROR,
//        authority.c_str());
    error_exists = true;
  }

  if (stripped.header().frame_id() == "") {
//    LOG_ERROR_FORMAT(
//        "%d%d TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from "
//        "authority \"%s\" because frame_id not set",
//        CYBERTRON_ERROR, TF_NO_FRAME_ID_ERROR,
//        stripped.child_frame_id().c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(stripped.transform().translation().x()) ||
      std::isnan(stripped.transform().translation().y()) ||
      std::isnan(stripped.transform().translation().z()) ||
      std::isnan(stripped.transform().rotation().qx()) ||
      std::isnan(stripped.transform().rotation().qy()) ||
      std::isnan(stripped.transform().rotation().qz()) ||
      std::isnan(stripped.transform().rotation().qw())) {
//    LOG_ERROR_FORMAT(
//        "%d%d TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from "
//        "authority \"%s\" because of a nan value in the transform (%f %f %f) "
//        "(%f %f %f %f)",
//        CYBERTRON_ERROR, TF_NAN_INPUT_ERROR,
//        stripped.child_frame_id().c_str(), authority.c_str(),
//        stripped.transform().translation().x(),
//        stripped.transform().translation().y(),
//        stripped.transform().translation().z(),
//        stripped.transform().rotation().qx(),
//        stripped.transform().rotation().qy(),
//        stripped.transform().rotation().qz(),
//        stripped.transform().rotation().qw());
    error_exists = true;
  }

  bool valid = std::abs((stripped.transform().rotation().qw() *
                             stripped.transform().rotation().qw() +
                         stripped.transform().rotation().qx() *
                             stripped.transform().rotation().qx() +
                         stripped.transform().rotation().qy() *
                             stripped.transform().rotation().qy() +
                         stripped.transform().rotation().qz() *
                             stripped.transform().rotation().qz()) -
                        1.0f) < 10e-6;

  if (!valid) {
//    LOG_ERROR_FORMAT(
//        "%d%d TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "
//        "\"%s\" from authority \"%s\" because of an invalid quaternion in the "
//        "transform (%f %f %f %f)",
//        CYBERTRON_ERROR, TF_DENORMALIZED_QUATERNION_ERROR,
//        stripped.child_frame_id().c_str(), authority.c_str(),
//        stripped.transform().rotation().qx(),
//        stripped.transform().rotation().qy(),
//        stripped.transform().rotation().qz(),
//        stripped.transform().rotation().qw());
    error_exists = true;
  }

  if (error_exists) return false;

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    CompactFrameID frame_number =
        lookupOrInsertFrameNumber(stripped.child_frame_id());
    TimeCacheInterfacePtr frame = getFrame(frame_number);
    if (frame == NULL) frame = allocateFrame(frame_number, is_static);

    if (frame->insertData(TransformStorage(
            stripped, lookupOrInsertFrameNumber(stripped.header().frame_id()),
            frame_number))) {
      frame_authority_[frame_number] = authority;
    } else {
//      LOG_WARN_FORMAT(
//          "TF_OLD_DATA ignoring data from the past for frame %s at time %lu "
//          "according to authority %s\nPossible reasons are listed at "
//          "http://wiki.cybertron.org/tf/Errors%%20explained",
//          stripped.child_frame_id().c_str(), stripped.header().stamp(),
//          authority.c_str());
      return false;
    }
  }

  testTransformableRequests();

  return true;
}

TimeCacheInterfacePtr BufferCore::allocateFrame(CompactFrameID cfid,
                                                bool is_static) {
  TimeCacheInterfacePtr frame_ptr = frames_[cfid];
  if (is_static) {
    frames_[cfid] = TimeCacheInterfacePtr(new StaticCache());
  } else {
    frames_[cfid] = TimeCacheInterfacePtr(new TimeCache(cache_time_));
  }

  return frames_[cfid];
}

enum WalkEnding {
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

// TODO for Jade: Merge walkToTopParent functions; this is now a stub to
// preserve ABI
template <typename F>
int BufferCore::walkToTopParent(F& f, cybertron::Time time,
                                CompactFrameID target_id,
                                CompactFrameID source_id,
                                std::string* error_string) const {
  return walkToTopParent(f, time, target_id, source_id, error_string, NULL);
}

template <typename F>
int BufferCore::walkToTopParent(
    F& f, cybertron::Time time, CompactFrameID target_id,
    CompactFrameID source_id, std::string* error_string,
    std::vector<CompactFrameID>* frame_chain) const {
  if (frame_chain) frame_chain->clear();

  // Short circuit if zero length transform to allow lookups on non existant
  // links
  if (source_id == target_id) {
    f.finalize(Identity, time);
    return 0;
    // return tf2_msgs::TF2Error::NO_ERROR;
  }

  // If getting the latest get the latest common time
  if (time == cybertron::Time()) {
    int retval = getLatestCommonTime(target_id, source_id, time, error_string);
    // if (retval != tf2_msgs::TF2Error::NO_ERROR)
    if (retval != 0) {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;

  std::string extrapolation_error_string;
  bool extrapolation_might_have_occurred = false;

  while (frame != 0) {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain) frame_chain->push_back(frame);

    if (!cache) {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, &extrapolation_error_string);
    if (parent == 0) {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      extrapolation_might_have_occurred = true;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id) {
      f.finalize(TargetParentOfSource, time);
      // return tf2_msgs::TF2Error::NO_ERROR;
      return 0;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      // return tf2_msgs::TF2Error::LOOKUP_ERROR;
      return 1;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its
  // transform
  frame = target_id;
  depth = 0;
  std::vector<CompactFrameID> reverse_frame_chain;

  while (frame != top_parent) {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain) reverse_frame_chain.push_back(frame);

    if (!cache) {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0) {
      if (error_string) {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame ["
           << lookupFrameString(source_id) << "] to frame ["
           << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      // return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      return 2;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id) {
      f.finalize(SourceParentOfTarget, time);
      if (frame_chain) {
        frame_chain->swap(reverse_frame_chain);
      }
      // return tf2_msgs::TF2Error::NO_ERROR;
      return 0;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      // return tf2_msgs::TF2Error::LOOKUP_ERROR;
      return 1;
    }
  }

  if (frame != top_parent) {
    if (extrapolation_might_have_occurred) {
      if (error_string) {
        std::stringstream ss;
        ss << extrapolation_error_string
           << ", when looking up transform from frame ["
           << lookupFrameString(source_id) << "] to frame ["
           << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      // return tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
      return 3;
    }

    createConnectivityErrorString(source_id, target_id, error_string);
    // return tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    return 2;
  }

  f.finalize(FullPath, time);
  if (frame_chain) {
    // Pruning: Compare the chains starting at the parent (end) until they
    // differ
    int m = reverse_frame_chain.size() - 1;
    int n = frame_chain->size() - 1;
    for (; m >= 0 && n >= 0; --m, --n) {
      if ((*frame_chain)[n] != reverse_frame_chain[m]) break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      frame_chain->erase(frame_chain->begin() + (n - 1), frame_chain->end());

    if (m < reverse_frame_chain.size()) {
      for (int i = m; i >= 0; --i) {
        frame_chain->push_back(reverse_frame_chain[i]);
      }
    }
  }

  // return tf2_msgs::TF2Error::NO_ERROR;
  return 0;
}

struct TransformAccum {
  TransformAccum()
      : source_to_top_quat(0.0, 0.0, 0.0, 1.0),
        source_to_top_vec(0.0, 0.0, 0.0),
        target_to_top_quat(0.0, 0.0, 0.0, 1.0),
        target_to_top_vec(0.0, 0.0, 0.0),
        result_quat(0.0, 0.0, 0.0, 1.0),
        result_vec(0.0, 0.0, 0.0) {}

  CompactFrameID gather(TimeCacheInterfacePtr cache, cybertron::Time time,
                        std::string* error_string) {
    if (!cache->getData(time, st, error_string)) {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source) {
    if (source) {
      source_to_top_vec =
          quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    } else {
      target_to_top_vec =
          quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, cybertron::Time _time) {
    switch (end) {
      case Identity:
        break;
      case TargetParentOfSource:
        result_vec = source_to_top_vec;
        result_quat = source_to_top_quat;
        break;
      case SourceParentOfTarget: {
        tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf2::Vector3 inv_target_vec =
            quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
      case FullPath: {
        tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf2::Vector3 inv_target_vec =
            quatRotate(inv_target_quat, -target_to_top_vec);

        result_vec =
            quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      } break;
    };

    time = _time;
  }

  TransformStorage st;
  cybertron::Time time;
  tf2::Quaternion source_to_top_quat;
  tf2::Vector3 source_to_top_vec;
  tf2::Quaternion target_to_top_quat;
  tf2::Vector3 target_to_top_vec;

  tf2::Quaternion result_quat;
  tf2::Vector3 result_vec;
};

::adu::common::TransformStamped BufferCore::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const cybertron::Time& time) const {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  if (target_frame == source_frame) {
    ::adu::common::TransformStamped identity;
    identity.mutable_header()->set_frame_id(target_frame);
    identity.set_child_frame_id(source_frame);
    identity.mutable_transform()->mutable_rotation()->set_qw(1);

    if (time == cybertron::Time()) {
      CompactFrameID target_id = lookupFrameNumber(target_frame);
      TimeCacheInterfacePtr cache = getFrame(target_id);
      if (cache)
        identity.mutable_header()->set_stamp(
            cache->getLatestTimestamp().ToNanosecond());
      else
        identity.mutable_header()->set_stamp(time.ToNanosecond());
    } else
      identity.mutable_header()->set_stamp(time.ToNanosecond());

    return identity;
  }

  // Identify case does not need to be validated above
  CompactFrameID target_id =
      validateFrameId("lookupTransform argument target_frame", target_frame);
  CompactFrameID source_id =
      validateFrameId("lookupTransform argument source_frame", source_frame);

  std::string error_string;
  TransformAccum accum;
  int retval =
      walkToTopParent(accum, time, target_id, source_id, &error_string);
  // if (retval != tf2_msgs::TF2Error::NO_ERROR)
  if (retval != 0) {
    switch (retval) {
      case 2:  // tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
        throw ConnectivityException(error_string);
      case 3:  // tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
        throw ExtrapolationException(error_string);
      case 1:  // tf2_msgs::TF2Error::LOOKUP_ERROR:
        throw LookupException(error_string);
      default:
//        LOG_ERROR_FORMAT("%d%d Unknown error code: %d", 
//            CYBERTRON_ERROR, TF_UNKNOWN_CODE_ERROR, retval);
        assert(0);
    }
  }

  ::adu::common::TransformStamped output_transform;
  transformTF2ToMsg(accum.result_quat, accum.result_vec, output_transform,
                    accum.time, target_frame, source_frame);
  return output_transform;
}

::adu::common::TransformStamped BufferCore::lookupTransform(
    const std::string& target_frame, const cybertron::Time& target_time,
    const std::string& source_frame, const cybertron::Time& source_time,
    const std::string& fixed_frame) const {
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

  ::adu::common::TransformStamped output;
  ::adu::common::TransformStamped temp1 =
      lookupTransform(fixed_frame, source_frame, source_time);
  ::adu::common::TransformStamped temp2 =
      lookupTransform(target_frame, fixed_frame, target_time);

  tf2::Transform tf1, tf2;
  transformMsgToTF2(temp1.transform(), tf1);
  transformMsgToTF2(temp2.transform(), tf2);
  transformTF2ToMsg(tf2 * tf1, *output.mutable_transform());
  output.mutable_header()->set_stamp(temp2.header().stamp());
  output.mutable_header()->set_frame_id(target_frame);
  output.set_child_frame_id(source_frame);
  return output;
}

/*
geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame,
                                          const std::string& observation_frame,
                                          const cybertron::Time& time,
                                          const cybertron::Time&
averaging_interval) const
{
  try
  {
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame,
                                          const std::string& observation_frame,
                                          const std::string& reference_frame,
                                          const tf2::Point & reference_point,
                                          const std::string&
reference_point_frame,
                                          const cybertron::Time& time,
                                          const cybertron::Time&
averaging_interval) const
{
  try{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame,
reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}
*/

struct CanTransformAccum {
  CompactFrameID gather(TimeCacheInterfacePtr cache, cybertron::Time time,
                        std::string* error_string) {
    return cache->getParent(time, error_string);
  }

  void accum(bool source) { (void)source; }

  void finalize(WalkEnding end, cybertron::Time _time) {
    (void)end;
    (void)_time;
  }

  TransformStorage st;
};

bool BufferCore::canTransformNoLock(CompactFrameID target_id,
                                    CompactFrameID source_id,
                                    const cybertron::Time& time,
                                    std::string* error_msg) const {
  if (target_id == 0 || source_id == 0) {
    return false;
  }

  if (target_id == source_id) {
    return true;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) ==
      0)  // tf2_msgs::TF2Error::NO_ERROR)
  {
    return true;
  }

  return false;
}

bool BufferCore::canTransformInternal(CompactFrameID target_id,
                                      CompactFrameID source_id,
                                      const cybertron::Time& time,
                                      std::string* error_msg) const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame,
                              const std::string& source_frame,
                              const cybertron::Time& time,
                              std::string* error_msg) const {
  // Short circuit if target_frame == source_frame
  if (target_frame == source_frame) return true;

  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;

  std::lock_guard<std::mutex> lock(frame_mutex_);

  CompactFrameID target_id = lookupFrameNumber(target_frame);
  CompactFrameID source_id = lookupFrameNumber(source_frame);

  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame,
                              const cybertron::Time& target_time,
                              const std::string& source_frame,
                              const cybertron::Time& source_time,
                              const std::string& fixed_frame,
                              std::string* error_msg) const {
  if (warnFrameId("canTransform argument target_frame", target_frame))
    return false;
  if (warnFrameId("canTransform argument source_frame", source_frame))
    return false;
  if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
    return false;

  return canTransform(target_frame, fixed_frame, target_time) &&
         canTransform(fixed_frame, source_frame, source_time, error_msg);
}

tf2::TimeCacheInterfacePtr BufferCore::getFrame(CompactFrameID frame_id) const {
  if (frame_id >= frames_.size())
    return TimeCacheInterfacePtr();
  else {
    return frames_[frame_id];
  }
}

CompactFrameID BufferCore::lookupFrameNumber(
    const std::string& frameid_str) const {
  CompactFrameID retval;
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end()) {
    retval = CompactFrameID(0);
  } else
    retval = map_it->second;
  return retval;
}

CompactFrameID BufferCore::lookupOrInsertFrameNumber(
    const std::string& frameid_str) {
  CompactFrameID retval = 0;
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end()) {
    retval = CompactFrameID(frames_.size());
    frames_.push_back(
        TimeCacheInterfacePtr());  // Just a place holder for iteration
    frameIDs_[frameid_str] = retval;
    frameIDs_reverse.push_back(frameid_str);
  } else
    retval = frameIDs_[frameid_str];

  return retval;
}

const std::string& BufferCore::lookupFrameString(
    CompactFrameID frame_id_num) const {
  if (frame_id_num >= frameIDs_reverse.size()) {
    std::stringstream ss;
    ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
    throw tf2::LookupException(ss.str());
  } else
    return frameIDs_reverse[frame_id_num];
}

void BufferCore::createConnectivityErrorString(CompactFrameID source_frame,
                                               CompactFrameID target_frame,
                                               std::string* out) const {
  if (!out) {
    return;
  }
  *out = std::string("Could not find a connection between '" +
                     lookupFrameString(target_frame) + "' and '" +
                     lookupFrameString(source_frame) +
                     "' because they are not part of the same tree." +
                     "Tf has two or more unconnected trees.");
}

std::string BufferCore::allFramesAsString() const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return this->allFramesAsStringNoLock();
}

std::string BufferCore::allFramesAsStringNoLock() const {
  std::stringstream mstream;

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it !=
  //  frames_.end(); ++it)

  /// regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter++) {
    TimeCacheInterfacePtr frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL) continue;
    CompactFrameID frame_id_num;
    if (frame_ptr->getData(cybertron::Time(), temp))
      frame_id_num = temp.frame_id_;
    else {
      frame_id_num = 0;
    }
    mstream << "Frame " << frameIDs_reverse[counter] << " exists with parent "
            << frameIDs_reverse[frame_id_num] << "." << std::endl;
  }

  return mstream.str();
}

struct TimeAndFrameIDFrameComparator {
  explicit TimeAndFrameIDFrameComparator(CompactFrameID id) : id(id) {}

  bool operator()(const P_TimeAndFrameID& rhs) const {
    return rhs.second == id;
  }

  CompactFrameID id;
};

int BufferCore::getLatestCommonTime(CompactFrameID target_id,
                                    CompactFrameID source_id,
                                    cybertron::Time& time,
                                    std::string* error_string) const {
  // Error if one of the frames don't exist.
  if (source_id == 0 || target_id == 0)
    return 0;  // tf2_msgs::TF2Error::LOOKUP_ERROR;

  if (source_id == target_id) {
    TimeCacheInterfacePtr cache = getFrame(source_id);
    // Set time to latest timestamp of frameid in case of target and source
    // frame id are the same
    if (cache)
      time = cache->getLatestTimestamp();
    else
      time = cybertron::Time();
    // return tf2_msgs::TF2Error::NO_ERROR;
    return 0;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of
  // parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  cybertron::Time common_time = cybertron::Time::MAX;  // cybertron::Time_MAX;
  while (frame != 0) {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache) {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0) {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (!latest.first.IsZero()) {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id) {
      time = common_time;
      // TODO CHEKC
      if (time == cybertron::Time::MAX) {
        time = cybertron::Time();
      }
      return 0;  // tf2_msgs::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return 1;  // tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest
  // time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = cybertron::Time::MAX;
  CompactFrameID common_parent = 0;
  while (true) {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache) {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0) {
      break;
    }

    if (!latest.first.IsZero()) {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it =
        std::find_if(lct_cache.begin(), lct_cache.end(),
                     TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache.end())  // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id) {
      time = common_time;
      if (time == cybertron::Time::MAX) {
        time = cybertron::Time();
      }
      return 0;  // tf2_msgs::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return 1;  // tf2_msgs::TF2Error::LOOKUP_ERROR;
    }
  }

  if (common_parent == 0) {
    createConnectivityErrorString(source_id, target_id, error_string);
    return 2;  // tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it) {
      if (!it->first.IsZero()) {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent) {
        break;
      }
    }
  }

  if (common_time == cybertron::Time::MAX) {
    common_time = cybertron::Time();
  }

  time = common_time;
  return 0;  // tf2_msgs::TF2Error::NO_ERROR;
}

std::string BufferCore::allFramesAsYAML(double current_time) const {
  std::stringstream mstream;
  std::lock_guard<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() == 1) mstream << "[]";

  mstream.precision(3);
  mstream.setf(std::ios::fixed, std::ios::floatfield);

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it !=
  //  frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size();
       counter++)  // one referenced for 0 is no frame
  {
    CompactFrameID cfid = CompactFrameID(counter);
    CompactFrameID frame_id_num;
    TimeCacheInterfacePtr cache = getFrame(cfid);
    if (!cache) {
      continue;
    }

    if (!cache->getData(cybertron::Time(), temp)) {
      continue;
    }

    frame_id_num = temp.frame_id_;

    std::string authority = "no recorded authority";
    std::map<CompactFrameID, std::string>::const_iterator it =
        frame_authority_.find(cfid);
    if (it != frame_authority_.end()) {
      authority = it->second;
    }

    double rate = cache->getListLength() /
                  std::max((cache->getLatestTimestamp().ToSecond() -
                            cache->getOldestTimestamp().ToSecond()),
                           0.0001);

    mstream << std::fixed;  // fixed point notation
    mstream.precision(3);   // 3 decimal places
    mstream << frameIDs_reverse[cfid] << ": " << std::endl;
    mstream << "  parent: '" << frameIDs_reverse[frame_id_num] << "'"
            << std::endl;
    mstream << "  broadcaster: '" << authority << "'" << std::endl;
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: " << (cache->getLatestTimestamp())
                                                  .ToSecond() << std::endl;
    mstream << "  oldest_transform: " << (cache->getOldestTimestamp())
                                             .ToSecond() << std::endl;
    if (current_time > 0) {
      mstream << "  transform_delay: "
              << current_time - cache->getLatestTimestamp().ToSecond()
              << std::endl;
    }
    mstream << "  buffer_length: "
            << (cache->getLatestTimestamp().ToNanosecond() -
                cache->getOldestTimestamp().ToNanosecond()) << std::endl;
  }

  return mstream.str();
}

std::string BufferCore::allFramesAsYAML() const {
  return this->allFramesAsYAML(0.0);
}

TransformableCallbackHandle BufferCore::addTransformableCallback(
    const TransformableCallback& cb) {
  std::lock_guard<std::mutex> lock(transformable_callbacks_mutex_);
  TransformableCallbackHandle handle = ++transformable_callbacks_counter_;
  while (!transformable_callbacks_.insert(std::make_pair(handle, cb)).second) {
    handle = ++transformable_callbacks_counter_;
  }

  return handle;
}

struct BufferCore::RemoveRequestByCallback {
  explicit RemoveRequestByCallback(TransformableCallbackHandle handle)
      : handle_(handle) {}

  bool operator()(const TransformableRequest& req) {
    return req.cb_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::removeTransformableCallback(
    TransformableCallbackHandle handle) {
  {
    std::lock_guard<std::mutex> lock(transformable_callbacks_mutex_);
    transformable_callbacks_.erase(handle);
  }

  {
    std::lock_guard<std::mutex> lock(transformable_requests_mutex_);
    V_TransformableRequest::iterator it = std::remove_if(
        transformable_requests_.begin(), transformable_requests_.end(),
        RemoveRequestByCallback(handle));
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}

TransformableRequestHandle BufferCore::addTransformableRequest(
    TransformableCallbackHandle handle, const std::string& target_frame,
    const std::string& source_frame, cybertron::Time time) {
  // shortcut if target == source
  if (target_frame == source_frame) {
    return 0;
  }

  TransformableRequest req;
  req.target_id = lookupFrameNumber(target_frame);
  req.source_id = lookupFrameNumber(source_frame);

  // First check if the request is already transformable.  If it is, return
  // immediately
  if (canTransformInternal(req.target_id, req.source_id, time, 0)) {
    return 0;
  }

  // Might not be transformable at all, ever (if it's too far in the past)
  if (req.target_id && req.source_id) {
    cybertron::Time latest_time;
    // TODO: This is incorrect, but better than nothing.  Really we want the
    // latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if (!latest_time.IsZero() && time + cache_time_ < latest_time) {
      return 0xffffffffffffffffULL;
    }
  }

  req.cb_handle = handle;
  req.time = time;
  req.request_handle = ++transformable_requests_counter_;
  if (req.request_handle == 0 || req.request_handle == 0xffffffffffffffffULL) {
    req.request_handle = 1;
  }

  if (req.target_id == 0) {
    req.target_string = target_frame;
  }

  if (req.source_id == 0) {
    req.source_string = source_frame;
  }

  std::lock_guard<std::mutex> lock(transformable_requests_mutex_);
  transformable_requests_.push_back(req);

  return req.request_handle;
}

struct BufferCore::RemoveRequestByID {
  explicit RemoveRequestByID(TransformableRequestHandle handle)
      : handle_(handle) {}

  bool operator()(const TransformableRequest& req) {
    return req.request_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::cancelTransformableRequest(TransformableRequestHandle handle) {
  std::lock_guard<std::mutex> lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it =
      std::remove_if(transformable_requests_.begin(),
                     transformable_requests_.end(), RemoveRequestByID(handle));

  if (it != transformable_requests_.end()) {
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}

// backwards compability for tf methods
cybertron::base::Connection<> BufferCore::_addTransformsChangedListener(
    std::function<void(void)> callback) {
  std::lock_guard<std::mutex> lock(transformable_requests_mutex_);
  return _transforms_changed_.Connect(callback);
}

void BufferCore::_removeTransformsChangedListener(
    cybertron::base::Connection<> c) {
  std::lock_guard<std::mutex> lock(transformable_requests_mutex_);
  if (!c.Disconnect()) {
    AINFO << "connection has disconnected.";
  }
}

bool BufferCore::_frameExists(const std::string& frame_id_str) const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return frameIDs_.count(frame_id_str);
}

bool BufferCore::_getParent(const std::string& frame_id, cybertron::Time time,
                            std::string& parent) const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  CompactFrameID frame_number = lookupFrameNumber(frame_id);
  TimeCacheInterfacePtr frame = getFrame(frame_number);

  if (!frame) return false;

  CompactFrameID parent_id = frame->getParent(time, NULL);
  if (parent_id == 0) return false;

  parent = lookupFrameString(parent_id);
  return true;
};

void BufferCore::_getFrameStrings(std::vector<std::string>& vec) const {
  vec.clear();

  std::lock_guard<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it !=
  //  frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frameIDs_reverse.size(); counter++) {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}

void BufferCore::testTransformableRequests() {
  std::unique_lock<std::mutex> lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it = transformable_requests_.begin();
  for (; it != transformable_requests_.end();) {
    TransformableRequest& req = *it;

    // One or both of the frames may not have existed when the request was
    // originally made.
    if (req.target_id == 0) {
      req.target_id = lookupFrameNumber(req.target_string);
    }

    if (req.source_id == 0) {
      req.source_id = lookupFrameNumber(req.source_string);
    }

    cybertron::Time latest_time;
    bool do_cb = false;
    TransformableResult result = TransformAvailable;
    // TODO: This is incorrect, but better than nothing.  Really we want the
    // latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if (!latest_time.IsZero() && req.time + cache_time_ < latest_time) {
      do_cb = true;
      result = TransformFailure;
    } else if (canTransformInternal(req.target_id, req.source_id, req.time,
                                    0)) {
      do_cb = true;
      result = TransformAvailable;
    }

    if (do_cb) {
      {
        std::lock_guard<std::mutex> lock2(transformable_callbacks_mutex_);
        M_TransformableCallback::iterator it =
            transformable_callbacks_.find(req.cb_handle);
        if (it != transformable_callbacks_.end()) {
          const TransformableCallback& cb = it->second;
          cb(req.request_handle, lookupFrameString(req.target_id),
             lookupFrameString(req.source_id), req.time, result);
        }
      }

      if (transformable_requests_.size() > 1) {
        transformable_requests_[it - transformable_requests_.begin()] =
            transformable_requests_.back();
      }

      transformable_requests_.erase(transformable_requests_.end() - 1);
    } else {
      ++it;
    }
  }

  // unlock before allowing possible user callbacks to avoid potential detadlock
  // (#91)
  lock.unlock();

  // Backwards compatability callback for tf
  _transforms_changed_();
}

std::string BufferCore::_allFramesAsDot(double current_time) const {
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  std::lock_guard<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() == 1) {
    mstream << "\"no tf data recieved\"";
  }
  mstream.precision(3);
  mstream.setf(std::ios::fixed, std::ios::floatfield);

  for (unsigned int counter = 1; counter < frames_.size();
       counter++)  // one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      continue;
    }
    if (!counter_frame->getData(cybertron::Time(), temp)) {
      continue;
    } else {
      frame_id_num = temp.frame_id_;
    }
    std::string authority = "no recorded authority";
    std::map<unsigned int, std::string>::const_iterator it =
        frame_authority_.find(counter);
    if (it != frame_authority_.end()) authority = it->second;

    double rate = counter_frame->getListLength() /
                  std::max((counter_frame->getLatestTimestamp().ToSecond() -
                            counter_frame->getOldestTimestamp().ToSecond()),
                           0.0001);

    mstream << std::fixed;  // fixed point notation
    mstream.precision(3);   // 3 decimal places
    mstream << "\"" << frameIDs_reverse[frame_id_num] << "\""
            << " -> "
            << "\"" << frameIDs_reverse[counter] << "\""
            << "[label=\""
            //<< "Time: " << current_time.ToSecond() << "\\n"
            << "Broadcaster: " << authority << "\\n"
            << "Average rate: " << rate << " Hz\\n"
            << "Most recent transform: "
            << (counter_frame->getLatestTimestamp()).ToSecond() << " ";
    if (current_time > 0)
      mstream << "( "
              << current_time - counter_frame->getLatestTimestamp().ToSecond()
              << " sec old)";
    mstream << "\\n"
            //    << "(time: " <<
            //    getFrame(counter)->getLatestTimestamp().ToSecond() << ")\\n"
            //    << "Oldest transform: " << (current_time -
            //    getFrame(counter)->getOldestTimestamp()).ToSecond() << " sec
            //    old \\n"
            //    << "(time: " <<
            //    (getFrame(counter)->getOldestTimestamp()).ToSecond() <<
            //    ")\\n"
            << "Buffer length: "
            << (counter_frame->getLatestTimestamp().ToNanosecond() -
                counter_frame->getOldestTimestamp().ToNanosecond()) << " sec\\n"
            << "\"];" << std::endl;
  }

  for (unsigned int counter = 1; counter < frames_.size();
       counter++)  // one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      if (current_time > 0) {
        mstream << "edge [style=invis];" << std::endl;
        mstream << " subgraph cluster_legend { style=bold; color=black; label "
                   "=\"view_frames Result\";\n"
                << "\"Recorded at time: " << current_time
                << "\"[ shape=plaintext ] ;\n "
                << "}"
                << "->"
                << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
      }
      continue;
    }
    if (counter_frame->getData(cybertron::Time(), temp)) {
      frame_id_num = temp.frame_id_;
    } else {
      frame_id_num = 0;
    }

    if (frameIDs_reverse[frame_id_num] == "NO_PARENT") {
      mstream << "edge [style=invis];" << std::endl;
      mstream << " subgraph cluster_legend { style=bold; color=black; label "
                 "=\"view_frames Result\";\n";
      if (current_time > 0)
        mstream << "\"Recorded at time: " << current_time
                << "\"[ shape=plaintext ] ;\n ";
      mstream << "}"
              << "->"
              << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}

std::string BufferCore::_allFramesAsDot() const { return _allFramesAsDot(0.0); }

void BufferCore::_chainAsVector(const std::string& target_frame,
                                cybertron::Time target_time,
                                const std::string& source_frame,
                                cybertron::Time source_time,
                                const std::string& fixed_frame,
                                std::vector<std::string>& output) const {
  std::string error_string;

  output.clear();  // empty vector

  std::stringstream mstream;
  std::lock_guard<std::mutex> lock(frame_mutex_);

  TransformAccum accum;

  // Get source frame/time using getFrame
  CompactFrameID source_id = lookupFrameNumber(source_frame);
  CompactFrameID fixed_id = lookupFrameNumber(fixed_frame);
  CompactFrameID target_id = lookupFrameNumber(target_frame);

  std::vector<CompactFrameID> source_frame_chain;
  int retval = walkToTopParent(accum, source_time, fixed_id, source_id,
                               &error_string, &source_frame_chain);

  if (retval != 0)  // tf2_msgs::TF2Error::NO_ERROR)
  {
    switch (retval) {
      case 2:  // tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
        throw ConnectivityException(error_string);
      case 3:  // tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
        throw ExtrapolationException(error_string);
      case 1:  // tf2_msgs::TF2Error::LOOKUP_ERROR:
        throw LookupException(error_string);
      default:
//        LOG_ERROR_FORMAT("%d%d Unknown error code: %d",
//            CYBERTRON_ERROR, TF_UNKNOWN_CODE_ERROR, retval);
        assert(0);
    }
  }
  if (source_time != target_time) {
    std::vector<CompactFrameID> target_frame_chain;
    retval = walkToTopParent(accum, target_time, target_id, fixed_id,
                             &error_string, &target_frame_chain);

    if (retval != 0)  // tf2_msgs::TF2Error::NO_ERROR)
    {
      switch (retval) {
        case 2:  // tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
          throw ConnectivityException(error_string);
        case 3:  // tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
          throw ExtrapolationException(error_string);
        case 1:  // tf2_msgs::TF2Error::LOOKUP_ERROR:
          throw LookupException(error_string);
        default:
//          LOG_ERROR_FORMAT("%d%d Unknown error code: %d",
//            CYBERTRON_ERROR, TF_UNKNOWN_CODE_ERROR, retval);
          assert(0);
      }
    }
    int m = target_frame_chain.size() - 1;
    int n = source_frame_chain.size() - 1;
    for (; m >= 0 && n >= 0; --m, --n) {
      if (source_frame_chain[n] != target_frame_chain[m]) break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      source_frame_chain.erase(source_frame_chain.begin() + (n - 1),
                               source_frame_chain.end());

    if (m < 0 || (unsigned)m < target_frame_chain.size()) {
      for (int i = 0; i <= m; ++i) {
        source_frame_chain.push_back(target_frame_chain[i]);
      }
    }
  }

  // Write each element of source_frame_chain as string
  for (uint32_t i = 0; i < source_frame_chain.size(); ++i) {
    output.push_back(lookupFrameString(source_frame_chain[i]));
  }
}

void BufferCore::setCacheTime(cybertron::Duration cache_time) {
  if (cache_time > cache_time_) {
    cache_time_ = cache_time;
  } 

}

}  // namespace tf2
}  // namespace tf2
}  // namespace tf2
