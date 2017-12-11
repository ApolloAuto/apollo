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
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "@glog//:glog",
    ],
)

cpplint()
