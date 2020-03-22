load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_pyramid_map_base_map_config",
    srcs = ["base_map_config.cc"],
    hdrs = [
        "base_map_config.h",
        "base_map_fwd.h",
    ],
    deps = [
        "//cyber/common",
        "//modules/common/util",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map_matrix",
    srcs = ["base_map_matrix.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_matrix.h",
    ],
    linkopts = [
        "-lz",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//cyber/common",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map_node_index",
    srcs = ["base_map_node_index.cc"],
    hdrs = [
        "base_map_config.h",
        "base_map_fwd.h",
        "base_map_node_index.h",
    ],
    deps = [
        ":localization_pyramid_map_base_map_config",
        "//cyber/common",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map_node_config",
    srcs = ["base_map_node_config.cc"],
    hdrs = ["base_map_node_config.h"],
    deps = [
        ":localization_pyramid_map_base_map_node_index",
        "//cyber/common",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map_node",
    srcs = ["base_map_node.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_matrix_handler.h",
        "base_map_node.h",
    ],
    deps = [
        ":localization_pyramid_map_base_map_config",
        ":localization_pyramid_map_base_map_matrix",
        ":localization_pyramid_map_base_map_node_config",
        ":localization_pyramid_map_base_map_node_index",
        "//cyber/common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/common/util:localization_msf_common_util_file_utility",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map_pool",
    srcs = ["base_map_pool.cc"],
    hdrs = [
        "base_map_fwd.h",
        "base_map_pool.h",
    ],
    deps = [
        ":localization_pyramid_map_base_map_config",
        ":localization_pyramid_map_base_map_node",
        ":localization_pyramid_map_base_map_node_index",
        "//cyber/common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "localization_pyramid_map_base_map",
    srcs = ["base_map.cc"],
    hdrs = [
        "base_map.h",
        "base_map_cache.h",
        "base_map_fwd.h",
    ],
    deps = [
        ":localization_pyramid_map_base_map_config",
        ":localization_pyramid_map_base_map_node",
        ":localization_pyramid_map_base_map_node_index",
        ":localization_pyramid_map_base_map_pool",
        "//cyber/common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_file_utility",
        "@com_google_glog//:glog",
        "@eigen",
    ],
)

cpplint()
