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
          "//framework/cybertron/class_loader:class_loader_lib",
          "//framework/cybertron/scheduler:scheduler_lib",
          "//framework/cybertron/node:node_lib",
          "//framework/cybertron/blocker:blocker_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

cpplint()
