load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io_util",
    srcs = ["io_util.cc"],
    hdrs = ["io_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base:camera",
        "//modules/perception/common/base:distortion_model",
        "//modules/perception/common/base:omnidirectional_model",
        "//modules/perception/common/algorithm/geometry:basic",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

# cc_test(
#     name = "io_util_test",
#     size = "small",
#     srcs = ["io_util_test.cc"],
#     copts = ["-fno-access-control"],
#     data = [
#         "//modules/perception/data:perception_testdata",
#     ],
#     deps = [
#         ":io_util",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
