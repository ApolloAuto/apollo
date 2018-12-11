load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libtimer_component_sample.so",
    deps = [":timer_component_sample_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "timer_component_sample_lib",
    srcs = [
        "timer_component_sample.cc",
    ],
    hdrs = [
        "timer_component_sample.h",
    ],
    deps = [
        "//cyber",
        "//cyber/samples/proto:samples_cc_proto",
    ],
)
cpplint()
