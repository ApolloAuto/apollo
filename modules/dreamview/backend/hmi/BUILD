load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hmi_status_helper",
    srcs = ["hmi_status_helper.cc"],
    hdrs = ["hmi_status_helper.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/dreamview/proto:hmi_status_proto",
    ],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
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
