#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
#define MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
#include <map>
#include <mutex>

#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

class ObjectSequence {
public:
    // typedef int TrackIdKey;
    // typedef double TimeStampKey;
    typedef std::map<double, ObjectPtr> TrackedObjects;
    // typedef std::map<TimeStampKey, ObjectPtr> LastSightingObjects;
public:
    ObjectSequence() = default;
    ~ObjectSequence() = default;

    bool add_tracked_frame_objects(
            const std::vector<ObjectPtr>& objects,
            double timestamp);

    bool get_track_in_temporal_window(
            int track_id, 
            TrackedObjects* track, 
            double window_time);

    // bool get_objects_in_spatial_area(
    //         LastSightingObjects* objects,
    //         const Eigen::Vector3d& center,
    //         double radius);

protected:
    void remove_stale_tracks(double current_stamp);
protected:
    double _current;
    std::map<int, TrackedObjects> _sequence;
    std::mutex _mutex;
    static constexpr double _s_max_time_out = 5; // 5 second
};

} // namespace perception
} // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
