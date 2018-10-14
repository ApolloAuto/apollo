load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "component_lib",
    srcs = glob([
        "timer_component.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/class_loader:class_loader_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/node:node_lib",
          "//cybertron/blocker:blocker_lib",
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
        "@gtest//:main",
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
        "@gtest//:main",
    ],
)

cpplint()
