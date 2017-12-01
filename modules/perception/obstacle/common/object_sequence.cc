#include "modules/perception/obstacle/common/object_sequence.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

bool ObjectSequence::AddTrackedFrameObjects(
    const std::vector<ObjectPtr>& objects, double timestamp) {
  std::lock_guard<std::mutex> lock(_mutex);
  for (const auto& obj : objects) {
    int& track_id = obj->track_id;
    auto iter = _sequence.find(track_id);
    if (iter == _sequence.end()) {
      auto res = _sequence.insert(std::make_pair(track_id, TrackedObjects()));
      if (!res.second) {
        AERROR << "Fail to insert track.";
        return false;
      }
      iter = res.first;
    }
    auto res = iter->second.insert(std::make_pair(timestamp, obj));
    if (!res.second) {
      AERROR << "Fail to insert object.";
      return false;
    }
  }
  RemoveStaleTracks(timestamp);
  _current = timestamp;
  return true;
}

bool ObjectSequence::GetTrackInTemporalWindow(int track_id,
                                              TrackedObjects* track,
                                              double window_time) {
  if (track == nullptr) {
    return false;
  }
  track->clear();
  std::lock_guard<std::mutex> lock(_mutex);
  double start_time = _current - window_time;
  auto iter = _sequence.find(track_id);
  if (iter == _sequence.end()) {
    return false;
  }
  for (auto& tobj : iter->second) {
    if (tobj.first >= start_time) {
      track->insert(tobj);
    }
  }
  return true;
}

// bool ObjectSequence::get_objects_in_spatial_area(
//         LastSightingObjects* objects,
//         const Eigen::Vector3d& center,
//         double radius) {
//     if (objects == nullptr) {
//         return false;
//     }
//     objects->clear();
//     std::lock_guard<std::mutex> lock(_mutex);
//     for (auto& track : _sequence) {
//         auto iter = track.second.rbegin();
//         auto obj_ptr = iter->second;
//         auto timestamp = iter->first;
//         if ((obj_ptr->center.head<2>() - center.head<2>()).norm() <= radius)
//         {
//             objects->insert(std::make_pair(timestamp, obj_ptr));
//         }
//     }
//     return true;
// }

void ObjectSequence::RemoveStaleTracks(double current_stamp) {
  for (auto outer_iter = _sequence.begin(); outer_iter != _sequence.end();) {
    CHECK(outer_iter->second.size() > 0) << "Find empty tracks.";
    auto& track = outer_iter->second;
    if (current_stamp - track.rbegin()->first > _s_max_time_out) {
      _sequence.erase(outer_iter++);
      continue;
    }
    for (auto inner_iter = track.begin(); inner_iter != track.end();) {
      if (current_stamp - inner_iter->first > _s_max_time_out) {
        track.erase(inner_iter++);
        continue;
      } else {
        break;
      }
    }
    if (track.size() == 0) {  // all element removed
      _sequence.erase(outer_iter++);
    } else {
      ++outer_iter;
    }
  }
}

}  // namespace perception
}  // namespace apollo
