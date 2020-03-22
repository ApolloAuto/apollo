load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_pyramid_map_ndt_map_config",
    srcs = ["ndt_map_config.cc"],
    hdrs = ["ndt_map_config.h"],
    copts = ["-g"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "@com_google_glog//:glog",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map_matrix",
    srcs = ["ndt_map_matrix.cc"],
    hdrs = ["ndt_map_matrix.h"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        ":localization_pyramid_map_ndt_map_config",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_matrix",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map_matrix_handler",
    srcs = ["ndt_map_matrix_handler.cc"],
    hdrs = ["ndt_map_matrix_handler.h"],
    deps = [
        ":localization_pyramid_map_ndt_map_config",
        ":localization_pyramid_map_ndt_map_matrix",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map_node_config",
    srcs = ["ndt_map_node_config.cc"],
    hdrs = ["ndt_map_node_config.h"],
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node_config",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map_node",
    srcs = ["ndt_map_node.cc"],
    hdrs = ["ndt_map_node.h"],
    deps = [
        ":localization_pyramid_map_ndt_map_config",
        ":localization_pyramid_map_ndt_map_matrix",
        ":localization_pyramid_map_ndt_map_matrix_handler",
        ":localization_pyramid_map_ndt_map_node_config",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_node",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map_pool",
    srcs = ["ndt_map_pool.cc"],
    hdrs = ["ndt_map_pool.h"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        ":localization_pyramid_map_ndt_map_matrix",
        ":localization_pyramid_map_ndt_map_matrix_handler",
        ":localization_pyramid_map_ndt_map_node",
        ":localization_pyramid_map_ndt_map_node_config",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_config",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map_pool",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_ndt_map",
    srcs = ["ndt_map.cc"],
    hdrs = ["ndt_map.h"],
    deps = [
        ":localization_pyramid_map_ndt_map_config",
        ":localization_pyramid_map_ndt_map_node",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

filegroup(
    name = "localization_pyramid_map_ndt_map_test_data",
    srcs = glob([
        "local_map/test_data/ndt_map/**",
    ]),
)

cc_test(
    name = "localization_pyramid_map_ndt_map_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_test.cc"],
    data = [":localization_pyramid_map_ndt_map_test_data"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map_pool",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "localization_pyramid_map_ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_map_config_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map_config",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cpplint()
