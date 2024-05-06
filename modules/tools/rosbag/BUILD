load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     py_dest = "tools/rosbag",
#     targets = [
#         ":audio_event_record",
#         ":body_sensation_evaluation",
#         ":channel_size_stats",
#         ":drive_event",
#         ":dump_gpsbin",
#         ":dump_planning",
#         ":dump_record",
#         ":dump_road_test_log",
#         ":extract_trajectory",
#         ":sample_pnc_topics",
#         ":stat_mileage",
#         ":stat_static_info",
#         ":tf_stats",
#         ":transcribe",
#     ],
# )

apollo_py_binary(
    name = "audio_event_record",
    srcs = ["audio_event_record.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "body_sensation_evaluation",
    srcs = ["body_sensation_evaluation.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "channel_size_stats",
    srcs = ["channel_size_stats.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_binary(
    name = "drive_event",
    srcs = ["drive_event.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "ros_dump_gpsbin",
    srcs = ["ros_dump_gpsbin.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:gnss_py_pb2",
    ],
)

apollo_py_binary(
    name = "dump_planning",
    srcs = ["dump_planning.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
    ],
)

apollo_py_binary(
    name = "dump_record",
    srcs = ["dump_record.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/common:message_manager",
    ],
)

apollo_py_binary(
    name = "dump_road_test_log",
    srcs = ["dump_road_test_log.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/basic_msgs:drive_event_py_pb2",
    ],
)

apollo_py_binary(
    name = "extract_trajectory",
    srcs = ["extract_trajectory.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/common:message_manager",
    ],
)

apollo_py_binary(
    name = "sample_pnc_topics",
    srcs = ["sample_pnc_topics.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "stat_mileage",
    srcs = ["stat_mileage.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "stat_static_info",
    srcs = ["stat_static_info.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/dreamview_msgs:hmi_status_py_pb2",
    ],
)

apollo_py_binary(
    name = "tf_stats",
    srcs = ["tf_stats.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/transform_msgs:transform_py_pb2",
    ],
)

apollo_py_binary(
    name = "transcribe",
    srcs = ["transcribe.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_package()