load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "hdmap_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_common_hdmap",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map:apollo_map",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
    ],
)

#apollo_cc_test(
#    name = "hdmap_input_test",
#    size = "small",
#    srcs = ["hdmap_input_test.cc"],
#    deps = [
#        ":hdmap_input",
#        "@com_google_googletest//:gtest_main",
#        "//modules/perception/common/base:apollo_perception_common_base",
#    ],
#)

apollo_package()
cpplint()
