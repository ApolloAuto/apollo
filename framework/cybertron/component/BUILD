load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "component",
    hdrs = [
        "component.h",
    ],
    deps = [
          "//cybertron/component:component_base",
    ],
)

cc_test(
    name = "component_test",
    size = "small",
    srcs = [
        "component_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/component:component",
        "@gtest//:main",
    ],
)

cc_library(
    name = "timer_component",
    srcs = [
        "timer_component.cpp",
    ],
    hdrs = [
        "timer_component.h",
    ],
    deps = [
          "//cybertron/component:component_base",
    ],
)

cc_test(
    name = "timer_component_test",
    size = "small",
    srcs = [
        "timer_component_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/component:timer_component",
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
          "//cybertron/blocker:blocker_lib",
          "//cybertron/class_loader:class_loader_lib",
          "//cybertron/node:node_lib",
          "//cybertron/scheduler:scheduler_lib",
    ],
)

cpplint()
