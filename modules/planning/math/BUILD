load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "double",
    srcs = [
        "double.cc",
    ],
    hdrs = glob([
        "double.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cpplint()
