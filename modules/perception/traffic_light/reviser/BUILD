load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_reviser",
    srcs = [
        "color_decision.cc",
    ],
    hdrs = [
        "color_decision.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
    ],
)

cpplint()
