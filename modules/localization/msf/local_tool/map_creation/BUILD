load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lossless_map_creator",
    srcs = ["lossless_map_creator.cc"],
    linkopts = [
        "-llz4",
    ],
    linkstatic = 0,
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map_pool",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = ["lossless_map_to_lossy_map.cc"],
    linkstatic = 0,
    deps = [
        "//cyber",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map_pool",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_binary(
    name = "poses_interpolator",
    srcs = ["poses_interpolator.cc"],
    linkstatic = 0,
    deps = [
        "//modules/localization/msf/local_tool/map_creation/poses_interpolation:poses_interpolation_lib",
        "@eigen",
    ],
)

cpplint()
