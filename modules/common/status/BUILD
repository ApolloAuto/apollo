load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(
    default_visibility = ["//visibility:public"],
)

apollo_cc_library(
    name = "status",
    hdrs = ["status.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

apollo_cc_test(
    name = "status_test",
    size = "small",
    srcs = ["status_test.cc"],
    deps = [
        ":status",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
