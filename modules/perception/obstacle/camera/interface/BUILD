load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    srcs = [],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/common",
        "@eigen",
        "@opencv2//:core",
    ],
)

cpplint()
