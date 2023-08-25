load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "map_datachecker_server",
    srcs = ["main.cc"],
    deps = [
        ":apollo_map_tools_map_datachecker_server",
        "//cyber",
    ],
)

apollo_cc_library(
    name = "apollo_map_tools_map_datachecker_server",
    srcs = [
        "channel_verify.cc",
        "channel_verify_agent.cc",
        "common.cc",
        "eight_route.cc",
        "laps_checker.cc",
        "loops_verify_agent.cc",
        "pj_transformer.cc",
        "pose_collection.cc",
        "pose_collection_agent.cc",
        "static_align.cc",
        "worker.cc",
        "worker_agent.cc",
        "worker_cyber_node.cc",
        "worker_gflags.cc",
    ],
    hdrs = [
        "alignment.h",
        "alignment_agent.h",
        "channel_verify.h",
        "channel_verify_agent.h",
        "common.h",
        "eight_route.h",
        "laps_checker.h",
        "loops_verify_agent.h",
        "pj_transformer.h",
        "pose_collection.h",
        "pose_collection_agent.h",
        "static_align.h",
        "worker.h",
        "worker_agent.h",
        "worker_cyber_node.h",
        "worker_gflags.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_grpc_grpc//:grpc++",
        "@proj",
    ],
)

apollo_package()

cpplint()