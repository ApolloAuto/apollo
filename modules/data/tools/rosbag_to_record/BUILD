load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "rosbag_to_record",
    srcs = ["rosbag_to_record.cc"],
    deps = [
        ":channel_info",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "channel_info",
    srcs = [
        "channel_info.cc",
    ],
    hdrs = [
        "channel_info.h",
    ],
    deps = [
        "//cyber", 
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/monitor_log/proto:monitor_log_proto",
        "//modules/common/proto:drive_event_proto",
        "//modules/control/proto:control_proto",
        "//modules/data/proto:static_info_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/monitor/proto:system_status_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/transform/proto:transform_proto",
    ],
)

cpplint()
