load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    linkopts = ["-shared"],
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
)

cpplint()
