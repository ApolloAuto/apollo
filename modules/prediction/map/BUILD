load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_map",
    hdrs = [
        "prediction_map.h",
    ],
    srcs = [
        "prediction_map.cc",
    ],
    deps = [
        "//modules/map/hdmap:hdmap",
        "//modules/common:macro",
    ]
)

cpplint()
