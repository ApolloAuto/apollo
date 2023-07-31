load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_traffic_light_tracker",
    hdrs = ["base_traffic_light_tracker.h"],
    deps = [
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
    ],
)

apollo_package()
cpplint()
