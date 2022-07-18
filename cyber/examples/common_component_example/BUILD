load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    linkshared = True,
    linkstatic = True,
    deps = [":common_component_example_lib"],
)

cc_library(
    name = "common_component_example_lib",
    srcs = ["common_component_example.cc"],
    hdrs = ["common_component_example.h"],
    visibility = ["//visibility:private"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "channel_test_writer",
    srcs = ["channel_test_writer.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "channel_prediction_writer",
    srcs = ["channel_prediction_writer.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

filegroup(
    name = "conf",
    srcs = [
        ":common.dag",
        ":common.launch",
    ],
)

install(
    name = "install",
    data = [
        ":conf",
    ],
    # We want to install channel_*_writer into runtime_dest
    # rather than bin/
    runtime_dest = "cyber/examples/common_component_example",
    targets = [
        ":channel_prediction_writer",
        ":channel_test_writer",
        ":libcommon_component_example.so",
    ],
)

cpplint()
