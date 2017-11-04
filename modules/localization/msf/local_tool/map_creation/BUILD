load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lossless_map_creator",
    srcs = [
        "lossless_map_creator.cc"
    ],
    linkstatic = 0,
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = [
        "lossless_map_to_lossy_map.cc"
    ],
    linkstatic = 0,
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cpplint()
