load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_base",
    srcs = [
        "object.cc",
        "types.cc",
        "object_supplement.cpp"
    ],
    hdrs = [
        "hdmap_struct.h",
        "object.h",
        "types.h",
        "object_supplement.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/util:string_util",
        "//modules/common/proto:error_code_proto",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/proto:perception_proto",
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
