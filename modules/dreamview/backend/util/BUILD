load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "json_util",
    srcs = [
        "json_util.cc",
    ],
    hdrs = [
        "json_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//third_party/json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "json_util_test",
    size = "small",
    srcs = [
        "json_util_test.cc",
    ],
    deps = [
        ":json_util",
         "//modules/dreamview/proto:hmi_status_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "http_client",
    srcs = ["http_client.cc"],
    hdrs = ["http_client.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/status",
        "//third_party/json",
        "@curlpp//:curlpp",
    ],
)

cc_library(
    name = "trajectory_point_collector",
    srcs = [
        "trajectory_point_collector.cc",
    ],
    hdrs = [
        "trajectory_point_collector.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_test(
    name = "trajectory_point_collector_test",
    size = "small",
    srcs = [
        "trajectory_point_collector_test.cc",
    ],
    deps = [
        ":trajectory_point_collector",
        "@gtest//:main",
    ],
)

cpplint()
