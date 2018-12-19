load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_ndt_map",
    srcs = ["ndt_map.cc",
            "ndt_map_config.cc",
            "ndt_map_matrix.cc",
            "ndt_map_node.cc",
            "ndt_map_pool.cc",
           ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen",
    ],
)


cpplint()
