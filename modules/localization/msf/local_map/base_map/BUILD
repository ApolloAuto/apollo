load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_base_map",
    srcs = [
        "base_map.cc",
        "base_map_config.cc",
        "base_map_matrix.cc",
        "base_map_node.cc",
        "base_map_node_index.cc",
        "base_map_pool.cc",
    ],
    hdrs = glob(["*.h"]),
    linkopts = [
        "-lz",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//modules/common/util",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/common/util:localization_msf_common_util_system_utility",
        "@eigen",
    ],
)

cc_test(
    name = "localization_msf_local_map_cache_test",
    size = "medium",
    timeout = "short",
    srcs = ["map_cache_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//third_party:boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_msf_local_map_node_index_test",
    size = "medium",
    timeout = "short",
    srcs = ["map_node_index_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
