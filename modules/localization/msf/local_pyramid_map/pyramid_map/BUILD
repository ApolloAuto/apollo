load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_pyramid_map_pyramid_map_config",
    srcs = ["pyramid_map_config.cc"],
    hdrs = ["pyramid_map_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map_matrix",
    srcs = ["pyramid_map_matrix.cc"],
    hdrs = ["aligned_matrix.h",
            "pyramid_map_matrix.h"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        ":localization_pyramid_map_pyramid_map_config",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_matrix",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "@eigen",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map_matrix_handler",
    srcs = ["pyramid_map_matrix_handler.cc"],
    hdrs = ["pyramid_map_matrix_handler.h"],
           
    deps = [
        ":localization_pyramid_map_pyramid_map_matrix",
        ":localization_pyramid_map_pyramid_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node",
        "@eigen",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map_node_config",
    srcs = ["pyramid_map_node_config.cc"],
    hdrs = ["pyramid_map_node_config.h"],
           
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node_config",
        "@eigen",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map_node",
    srcs = ["pyramid_map_node.cc"],
    hdrs = ["pyramid_map_node.h"],
    deps = [
        ":localization_pyramid_map_pyramid_map_matrix",
        ":localization_pyramid_map_pyramid_map_config",
        ":localization_pyramid_map_pyramid_map_node_config",
        ":localization_pyramid_map_pyramid_map_matrix_handler",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node",
        "@eigen",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map_pool",
    srcs = ["pyramid_map_pool.cc"],
    hdrs = ["pyramid_map_pool.h"],
           
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        ":localization_pyramid_map_pyramid_map_node",
        ":localization_pyramid_map_pyramid_map_matrix",
        ":localization_pyramid_map_pyramid_map_node_config",
        ":localization_pyramid_map_pyramid_map_matrix_handler",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_pool",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "@eigen",
        "@glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_pyramid_map",
    srcs = ["pyramid_map.cc"],
    hdrs = ["pyramid_map.h"],
    deps = [
        ":localization_pyramid_map_pyramid_map_node",
        ":localization_pyramid_map_pyramid_map_config",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "@eigen",
        "@glog",
    ],
)
cc_test (
    name = "pyramid_map_config_test",
    srcs = ["pyramid_map_config_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map_config",
    ],
)

cc_test (
    name = "pyramid_map_matrix_handler_test",
    srcs = ["pyramid_map_matrix_handler_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map_matrix_handler",
    ],
)

cc_test (
    name = "pyramid_map_matrix_test",
    srcs = ["pyramid_map_matrix_test.cc"],
    size = "medium",
    timeout = "short",
    copts = ["-Dprivate=public"],
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map_matrix",
    ],
)

cc_test (
    name = "pyramid_map_node_test",
    srcs = ["pyramid_map_node_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map_node",
    ],
)

cc_test (
    name = "pyramid_map_pool_test",
    srcs = ["pyramid_map_pool_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map",
        ":localization_pyramid_map_pyramid_map_pool",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
    ],
)

cc_test (
    name = "pyramid_map_test",
    srcs = ["pyramid_map_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_pyramid_map_pyramid_map",
        ":localization_pyramid_map_pyramid_map_pool",
    ],
)

cpplint()
