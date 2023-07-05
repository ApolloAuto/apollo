load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    library_dest = "common/lib",
    targets = [
        ":libstatus.so",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "status",
    hdrs = ["status.h"],
    srcs = ["libstatus.so"],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common/util:util_tool",
        "@com_google_protobuf//:protobuf",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libstatus.so",
    srcs = ["status.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common/util:util_tool",
        "@com_google_protobuf//:protobuf",
    ],
    linkshared = True,
    linkstatic = True,
)

cc_test(
    name = "status_test",
    size = "small",
    srcs = ["status_test.cc"],
    deps = [
        ":status",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
