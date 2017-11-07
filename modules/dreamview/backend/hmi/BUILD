load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket",
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
    data = [
        "//modules/dreamview/conf:hmi_config",
    ],
    deps = [
        ":hmi",
        "@gtest//:main",
    ],
)

cpplint()
