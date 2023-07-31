load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_gatekeeper",
    srcs = ["pbf_gatekeeper.cc"],
    hdrs = ["pbf_gatekeeper.h"],
    deps = [
        "//modules/common/util",
        "//modules/perception/common:util",
        "//modules/perception/multi_sensor_fusion/base:scene",
        "//modules/perception/multi_sensor_fusion/base:track",
        "//modules/perception/multi_sensor_fusion/proto:pbf_gatekeeper_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/interface",
    ],
    alwayslink = True,
)

# todo(zero)
# cc_test(
#     name = "pbf_gatekeeper_test",
#     size = "small",
#     srcs = ["pbf_gatekeeper_test.cc"],
#     deps = [
#         ":pbf_gatekeeper",
#         "//modules/perception/common/sensor_manager",
#         "//modules/perception/fusion/base:sensor",
#         "//modules/perception/fusion/base:track",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
