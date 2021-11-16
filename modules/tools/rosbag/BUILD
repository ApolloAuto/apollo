load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "audio_event_record",
    srcs = ["audio_event_record.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "body_sensation_evaluation",
    srcs = ["body_sensation_evaluation.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "channel_size_stats",
    srcs = ["channel_size_stats.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_binary(
    name = "drive_event",
    srcs = ["drive_event.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "dump_gpsbin",
    srcs = ["dump_gpsbin.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/gnss/proto:gnss_py_pb2",
    ],
)

py_binary(
    name = "dump_planning",
    srcs = ["dump_planning.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
    ],
)

py_binary(
    name = "dump_record",
    srcs = ["dump_record.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/common:message_manager",
    ],
)

py_binary(
    name = "dump_road_test_log",
    srcs = ["dump_road_test_log.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common/proto:drive_event_py_pb2",
    ],
)

py_binary(
    name = "extract_trajectory",
    srcs = ["extract_trajectory.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/common:message_manager",
    ],
)

py_binary(
    name = "sample_pnc_topics",
    srcs = ["sample_pnc_topics.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "stat_mileage",
    srcs = ["stat_mileage.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "stat_static_info",
    srcs = ["stat_static_info.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/dreamview/proto:hmi_status_py_pb2",
    ],
)

py_binary(
    name = "tf_stats",
    srcs = ["tf_stats.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/transform/proto:transform_py_pb2",
    ],
)

py_binary(
    name = "transcribe",
    srcs = ["transcribe.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)
