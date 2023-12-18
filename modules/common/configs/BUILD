load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

apollo_cc_library(
    name = "config_gflags",
    srcs = ["config_gflags.cc"],
    hdrs = ["config_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_cc_library(
    name = "vehicle_config_helper",
    srcs = ["vehicle_config_helper.cc"],
    hdrs = ["vehicle_config_helper.h"],
    deps = [
        ":config_gflags",
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "vehicle_config_helper_test",
    size = "small",
    srcs = ["vehicle_config_helper_test.cc"],
    deps = [
        ":vehicle_config_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
