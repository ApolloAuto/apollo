load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_manager",
    srcs = ["container_manager.cc"],
    hdrs = ["container_manager.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "//modules/prediction/container/storytelling:storytelling_container",
        "@com_google_googletest//:gtest_main",
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
        "//modules/prediction/container:container_manager",
    ],
)

cc_library(
    name = "container",
    hdrs = ["container.h"],
)

cpplint()
