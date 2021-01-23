load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CURRENT_PKG = "cyber/examples/common_component_example"

cc_binary(
    name = "libcommon_component_example.so",
    linkshared = True,
    linkstatic = False,
    deps = [":common_component_example_lib"],
)

cc_library(
    name = "common_component_example_lib",
    srcs = ["common_component_example.cc"],
    hdrs = ["common_component_example.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
    visibility = ["//visibility:private"],
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
    data_dest = CURRENT_PKG,
    targets = [
        ":channel_prediction_writer",
        ":channel_test_writer",
        ":libcommon_component_example.so",
    ],
    library_dest = CURRENT_PKG,
    runtime_dest = CURRENT_PKG,
)

cpplint()
