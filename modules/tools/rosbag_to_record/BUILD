load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "rosbag_to_record",
    srcs = ["rosbag_to_record.cc"],
    linkopts = [
       "-lprotobuf",
    ],
    deps = [
        ":rosbag_to_record_lib",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "rosbag_to_record_lib",
    srcs = [
        "channel_info.cc",
    ],
    hdrs = [
        "rosbag_to_record.h",
        "channel_info.h",
    ],
    deps = [
        "//framework:cybertron", 
        "//modules/planning/proto:planning_proto",
        "//modules/control/proto:control_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/common/proto:drive_event_proto",
        "//modules/monitor/proto:system_status_proto",
        "//modules/data/proto:static_info_proto",
        "//modules/common/monitor_log/proto:monitor_log_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/transform/proto:transform_proto",
    ],
)

cpplint()
