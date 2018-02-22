load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_base",
    srcs = [
        "object.cc",
        "types.cc",
    ],
    hdrs = [
        "hdmap_struct.h",
        "object.h",
        "object_supplement.h",
        "types.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/proto:error_code_proto",
        "//modules/common/util:string_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/proto:perception_proto",
        "@eigen",
        "@opencv2//:core",
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
