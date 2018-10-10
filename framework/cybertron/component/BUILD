load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "component_lib",
    srcs = glob([
        "*.cpp",
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

cpplint()
