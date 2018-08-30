load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_analyzer",
    srcs = ["scenario_analyzer.cc"],
    hdrs = ["scenario_analyzer.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/proto:scenario_proto",
        "//modules/prediction/proto:scenario_feature_proto",
    ],
)

cpplint()
