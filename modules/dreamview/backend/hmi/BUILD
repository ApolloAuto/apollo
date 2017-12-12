load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_manager",
    srcs = ["vehicle_manager.cc"],
    hdrs = ["vehicle_manager.h"],
    deps = [
        "//modules/common/util",
        "//modules/common/util:string_util",
        "//modules/dreamview/proto:hmi_config_proto",
    ],
)

cc_binary(
    name = "vehicle_manager_main",
    srcs = ["vehicle_manager_main.cc"],
    deps = [
        ":vehicle_manager",
    ],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    deps = [
        ":vehicle_manager",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:http_client",
        "//modules/common/util:json_util",
        "//modules/common/util:map_util",
        "//modules/data/proto:task_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/proto:hmi_config_proto",
        "//modules/dreamview/proto:hmi_status_proto",
        "@com_google_protobuf//:protobuf",
        "@gtest//:gtest",
    ],
)

cc_test(
    name = "hmi_test",
    size = "small",
    srcs = ["hmi_test.cc"],
    deps = [
        ":hmi",
        "@gtest//:main",
    ],
)

cpplint()
