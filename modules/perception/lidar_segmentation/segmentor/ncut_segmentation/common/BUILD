load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "flood_fill",
    srcs = ["flood_fill.cc"],
    hdrs = ["flood_fill.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
    ],
)

apollo_cc_library(
    name = "lr_classifier",
    hdrs = ["lr_classifier.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@eigen",
    ],
)

apollo_package(enable_source=False)
cpplint()
