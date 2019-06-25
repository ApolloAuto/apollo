load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_pyramid_map",
    srcs = ["pyramid_map_config.cc",
            "pyramid_map_matrix.cc",
            "pyramid_map_matrix_handler.cc"],

    hdrs = ["aligned_matrix.h",
            "pyramid_map_config.h",
            "pyramid_map_matrix.h",
            "pyramid_map_matrix_handler.h"],
           
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/pyramid_map/base_map:localization_msf_base_map",
        "@eigen",
    ],
)

cc_test (
    name = "pyramid_map_config_test",
    srcs = ["pyramid_map_config_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_msf_pyramid_map",
    ],
)

cc_test (
    name = "pyramid_map_matrix_handler_test",
    srcs = ["pyramid_map_matrix_handler_test.cc"],
    size = "medium",
    timeout = "short",
    deps = [
        "@gtest//:main",
        ":localization_msf_pyramid_map",
    ],
)

cc_test (
    name = "pyramid_map_matrix_test",
    srcs = ["pyramid_map_matrix_test.cc"],
    size = "medium",
    timeout = "short",
    copts = ["-Dprivate=public"],
    deps = [
        "@gtest//:main",
        ":localization_msf_pyramid_map",
    ],
)

cpplint()
