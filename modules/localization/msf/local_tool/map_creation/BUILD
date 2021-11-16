load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lossless_map_creator",
    srcs = ["lossless_map_creator.cc"],
    linkstatic = 0,
    deps = [
        "//cyber/common:file",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "@boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = ["lossless_map_to_lossy_map.cc"],
    linkstatic = 0,
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "@boost",
    ],
)

cc_binary(
    name = "poses_interpolator",
    srcs = ["poses_interpolator.cc"],
    linkstatic = 0,
    deps = [
        "//modules/localization/msf/local_tool/map_creation/poses_interpolation:poses_interpolation_lib",
    ],
)

cpplint()
