load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lossless_map_creator",
    srcs = [
        "lossless_map_creator.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common:log",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = [
        "lossless_map_to_lossy_map.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common:log",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "poses_interpolator",
    srcs = [
        "poses_interpolator.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        "//modules/localization/msf/local_tool/map_creation/poses_interpolation:poses_interpolation_lib",
        "@eigen//:eigen",
    ],
)

cpplint()
