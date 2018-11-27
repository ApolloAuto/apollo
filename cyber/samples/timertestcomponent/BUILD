load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libtimertest_component.so",
    deps = [":timertest_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "timertest_component_lib",
    srcs = [
        "timertest_component.cc",
    ],
    hdrs = [
        "timertest_component.h",
    ],
    deps = [
        "//cyber",
        "//cyber/samples/proto:samples_cc_proto",
    ],
)
cpplint()
