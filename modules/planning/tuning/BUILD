load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autotuning_raw_feature_generator",
    srcs = ["autotuning_raw_feature_generator.cc"],
    hdrs = ["autotuning_raw_feature_generator.h"],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:auto_tuning_raw_feature_proto",
    ],
)

cpplint()
