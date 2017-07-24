load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager/proto:config_proto",
        "//external:gflags",
        "@glog//:glog",
    ],
)

cpplint()
