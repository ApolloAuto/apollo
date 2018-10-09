load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scheduler_lib",
    srcs = glob([
        "*.cpp",
        "policy/*.cpp",
    ]),
    hdrs = glob([
        "policy/*.h",
        "*.h",
    ]),
    deps = [
          "//cybertron/croutine:croutine_lib",
          "//cybertron/data:data_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

cpplint()
