load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
      "//modules/perception/obstacle/base:perception_obstacle_base",
      "@eigen"
    ]
)

cpplint()
