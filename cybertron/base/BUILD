load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_lib",
    srcs = glob([
          "*.cpp",
    ]),
    hdrs = glob([
          "*.h",
    ]),
    deps = [
    ],
    copts = ["-Iframework"]
)

cpplint()
