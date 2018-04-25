load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sequence_type_fuser",
    srcs = [
        "fuser_util.cc",
        "sequence_type_fuser.cc",
    ],
    hdrs = [
        "base_type_fuser.h",
        "fuser_util.h",
        "sequence_type_fuser.h",
    ],
    deps = [
        ":object_sequence",
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common",
        "@eigen",
    ],
)

cc_library(
    name = "object_sequence",
    srcs = [
        "object_sequence.cc",
    ],
    hdrs = [
        "object_sequence.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
        "@eigen",
    ],
)

cc_test(
    name = "object_sequence_test",
    size = "small",
    srcs = [
        "object_sequence_test.cc",
    ],
    deps = [
        ":object_sequence",
        "@gtest//:main",
    ],
)

cc_test(
    name = "sequence_type_fuser_test",
    size = "small",
    srcs = [
        "sequence_type_fuser_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        ":sequence_type_fuser",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker",
        "@gtest//:main",
        "@pcl",
    ],
)

cpplint()
