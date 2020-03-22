load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "offline_local_visualizer_lib",
    srcs = ["offline_local_visualizer.cc"],
    hdrs = ["offline_local_visualizer.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = ["offline_local_visualizer_main.cc"],
    linkstatic = 0,
    deps = [
        ":offline_local_visualizer_lib",
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//third_party:boost",
        "@eigen",
    ],
)

cpplint()
