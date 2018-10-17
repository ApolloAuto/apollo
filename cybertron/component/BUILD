load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "component",
    hdrs = [
        "component.h",
    ],
    deps = [
        "component_base",
        "//cybertron/scheduler:scheduler",
    ],
)

cc_test(
    name = "component_test",
    size = "small",
    srcs = [
        "component_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_library(
    name = "timer_component",
    srcs = [
        "timer_component.cc",
    ],
    hdrs = [
        "timer_component.h",
    ],
    deps = [
        "component_base",
        "//cybertron/blocker:blocker_manager",
    ],
)

cc_test(
    name = "timer_component_test",
    size = "small",
    srcs = [
        "timer_component_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_library(
    name = "component_base",
    hdrs = [
        "component_base.h",
    ],
    deps = [
        "//cybertron/base:signal",
        "//cybertron/base:thread_pool",
        "//cybertron/class_loader:class_loader",
        "//cybertron/node:node_lib",
    ],
)

cpplint()
