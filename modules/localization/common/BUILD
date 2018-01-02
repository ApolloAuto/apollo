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
        "//modules/common:log",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/localization/proto:localization_proto",
    ],
)

cpplint()
