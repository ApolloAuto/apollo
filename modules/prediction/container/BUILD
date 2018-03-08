load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_manager",
    srcs = ["container_manager.cc"],
    hdrs = ["container_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "@glog//:glog",
    ],
)

cc_test(
    name = "container_manager_test",
    size = "small",
    srcs = ["container_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/container:container_manager",
        "@gtest//:main",
    ],
)

cc_library(
    name = "container",
    hdrs = ["container.h"],
    deps = [
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
