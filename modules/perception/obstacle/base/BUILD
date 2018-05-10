load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "object.cc",
        "object_supplement.cc",
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
        "//modules/common/time",
        "//modules/common/util:string_util",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/proto:perception_proto",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_test(
    name = "base_test",
    size = "small",
    srcs = [
        "object_test.cc",
        "types_test.cc",
    ],
    deps = [
        ":base",
        "@gtest//:main",
    ],
)

cpplint()
