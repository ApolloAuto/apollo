load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "feature_extractor",
    srcs = ["feature_extractor.cc"],
    hdrs = ["feature_extractor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/prediction/proto:scenario_feature_proto",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/pose:pose_container",
    ],
)

cpplint()
