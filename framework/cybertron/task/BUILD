#load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    copts = ["-Iframework"],
    deps = [
        "//framework/cybertron/scheduler:scheduler_lib",
    ],
)

#cpplint()
