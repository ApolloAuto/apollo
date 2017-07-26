load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_base",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/proto:perception_proto",
        "//modules/common:common",
        "//modules/common/util:string_util",
        "@eigen//:eigen",
    ],
)


cpplint()
