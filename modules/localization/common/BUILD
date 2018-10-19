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
        "//cyber",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/localization/proto:localization_proto",
    ],
)

cpplint()
