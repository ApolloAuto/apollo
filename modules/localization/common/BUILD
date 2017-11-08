load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_common",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/localization/proto:localization_proto",
        "//modules/common/util",
        "//modules/common/util:string_util",
        "@glog//:glog",
    ],
)

cpplint()
