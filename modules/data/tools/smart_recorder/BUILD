load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
    ]),
)

apollo_cc_library(
    name = "apollo_data_tools_smart_recorder",
    srcs = [
        "bumper_crash_trigger.cc",
        "channel_pool.cc",
        "drive_event_trigger.cc",
        "emergency_mode_trigger.cc",
        "hard_brake_trigger.cc",
        "interval_pool.cc",
        "post_record_processor.cc",
        "realtime_record_processor.cc",
        "record_processor.cc",
        "regular_interval_trigger.cc",
        "small_topics_trigger.cc",
        "smart_recorder_gflags.cc",
        "swerve_trigger.cc",
        "trigger_base.cc",
    ],
    hdrs = [
        "bumper_crash_trigger.h",
        "channel_pool.h",
        "drive_event_trigger.h",
        "emergency_mode_trigger.h",
        "hard_brake_trigger.h",
        "interval_pool.h",
        "post_record_processor.h",
        "realtime_record_processor.h",
        "record_processor.h",
        "regular_interval_trigger.h",
        "small_topics_trigger.h",
        "smart_recorder_gflags.h",
        "swerve_trigger.h",
        "trigger_base.h",
    ],
    deps = [
        "//cyber",
        "//cyber/tools/cyber_recorder:recorder",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:common_util",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/data/tools/smart_recorder/proto:smart_recorder_triggers_cc_proto",
        "//modules/monitor:apollo_monitor",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
    ],
)

apollo_cc_binary(
    name = "smart_recorder",
    srcs = ["smart_recorder.cc"],
    deps = [
        ":apollo_data_tools_smart_recorder",
    ],
)

apollo_package()

cpplint()
