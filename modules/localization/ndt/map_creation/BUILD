load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_map_creator",
    srcs = [
        "ndt_map_creator.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    linkstatic = 0,
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
