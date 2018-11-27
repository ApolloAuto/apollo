load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommontest_component.so",
    deps = [":commontest_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "commontest_component_lib",
    srcs = [
        "commontest_component.cc",
    ],
    hdrs = [
        "commontest_component.h",
    ],
    deps = [
        "//cyber",
        "//cyber/samples/proto:samples_cc_proto",
    ],
)
cpplint()
