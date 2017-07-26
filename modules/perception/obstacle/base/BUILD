load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_base",
    srcs = [
        "object.cc",
        "types.cc",
        ],
    hdrs = [
        "object.h",
        "types.h",
        "hdmap_struct.h",
        ],
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/proto:perception_proto",
        "//modules/common:common",
        "//modules/common/util:string_util",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "perception_obstacle_base_test",
    size = "small",
    srcs = [
        "object_test.cc",
        "types_test.cc",
    ],
    deps = [
        ":perception_obstacle_base",
        "@gtest//:main",
    ],
)

cpplint()
