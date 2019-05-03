load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "smart_recorder",
    srcs = ["smart_recorder.cc"],
    deps = [
        ":record_processor",
        ":smart_recorder_gflags",
    ],
)

cc_library(
    name = "record_processor",
    srcs = ["record_processor.cc"],
    hdrs = ["record_processor.h"],
    deps = [
        ":drive_event_trigger",
        ":emergency_mode_trigger",
        ":small_topics_trigger",
        "//cyber/common:file",
        "//cyber/message:message_traits",
        "//cyber/message:raw_message",
        "//cyber/record:record_reader",
        "//cyber/record:record_viewer",
        "//cyber/record:record_writer",
        "//modules/common/util",
    ],
)

cc_library(
    name = "smart_recorder_gflags",
    srcs = ["smart_recorder_gflags.cc"],
    hdrs = ["smart_recorder_gflags.h"],
    deps = ["//external:gflags"],
)

cc_library(
    name = "drive_event_trigger",
    srcs = ["drive_event_trigger.cc"],
    hdrs = ["drive_event_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:drive_event_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

cc_library(
    name = "emergency_mode_trigger",
    srcs = ["emergency_mode_trigger.cc"],
    hdrs = ["emergency_mode_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

cc_library(
    name = "small_topics_trigger",
    srcs = ["small_topics_trigger.cc"],
    hdrs = ["small_topics_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/transform/proto:transform_proto",
    ],
)

cc_library(
    name = "trigger_base",
    srcs = ["trigger_base.cc"],
    hdrs = ["trigger_base.h"],
    deps = [
        ":interval_pool",
        "//cyber/record:record_message",
        "//modules/data/tools/smart_recorder/proto:smart_recorder_triggers_proto",
    ],
)

cc_library(
    name = "interval_pool",
    srcs = ["interval_pool.cc"],
    hdrs = ["interval_pool.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/util",
    ],
)

cpplint()
