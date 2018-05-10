load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_common",
    srcs = [
        "canbus_gflags.cc",
    ],
    hdrs = [
        "canbus_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
