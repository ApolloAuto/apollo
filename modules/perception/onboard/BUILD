load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onboard",
    srcs = [
        "common_shared_data.cc",
        "dag_streaming.cc",
        "event_manager.cc",
        "shared_data_manager.cc",
        "subnode.cc",
        "subnode_helper.cc",
        "transform_input.cc",
    ],
    hdrs = [
        "common_shared_data.h",
        "dag_streaming.h",
        "event_manager.h",
        "shared_data.h",
        "shared_data_manager.h",
        "subnode.h",
        "subnode_helper.h",
        "transform_input.h",
        "types.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/onboard/proto:dag_proto",
        "@eigen",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "subnode_test",
    size = "small",
    srcs = [
        "subnode_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        ":onboard",
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "@gtest",
        "@gtest//:main",
    ],
)

cpplint()
