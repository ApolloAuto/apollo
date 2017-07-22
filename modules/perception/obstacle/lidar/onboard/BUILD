package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_onboard",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/lidar/interface:interface",
        "//modules/perception/obstacle/common:common",
        "//modules/perception/lib/base:base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/common/proto:common_proto",
        "//modules/common:log",
    ]
)
