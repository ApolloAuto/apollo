load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scheduler",
    srcs = [
        "processor.cc",
        "processor_context.cc",
        "scheduler.cc",
        "policy/scheduler_choreography.cc",
        "policy/scheduler_classic.cc",
        "policy/choreography.cc",
        "policy/classic.cc",
    ],
    hdrs = [
        "processor.h",
        "processor_context.h",
        "scheduler.h",
        "policy/scheduler_choreography.h",
        "policy/scheduler_classic.h",
        "policy/choreography.h",
        "policy/classic.h",
    ],
    deps = [
        "//cyber/croutine:croutine",
        "//cyber/data:data",
        "//cyber/proto:component_conf_cc_proto",
    ],
)

cc_test(
    name = "scheduler_test",
    size = "small",
    srcs = [
        "scheduler_test.cc",
    ],
    deps = [
        "//cyber",
        "@gtest//:main",
    ],
)

cc_test(
    name = "scheduler_policy_test",
    size = "small",
    srcs = [
        "scheduler_policy_test.cc",
    ],
    deps = [
        "//cyber",
        "@gtest//:main",
    ],
)

cpplint()
