load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/prediction/proto:prediction_conf_proto",
    ],
)

cpplint()
