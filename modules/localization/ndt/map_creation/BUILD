load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_map_creator",
    srcs = ["ndt_map_creator.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_system_utility",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map_pool",
        "@boost",
        "@eigen",
        "@opencv",
        "@pcl",
    ],
)

cpplint()
