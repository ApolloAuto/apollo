load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    srcs = [],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/base:base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/camera/common:common",
        "@eigen",
        "@opencv2//:core",
    ]
)

cpplint()
