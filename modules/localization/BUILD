load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_base",
    hdrs = [
        "localization_base.h",
    ],
)

cc_library(
    name = "localization_app",
    srcs = [
        "localization.cc",
    ],
    hdrs = [
        "localization.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:apollo_app",
        "//modules/common/util:factory",
        "//modules/localization:localization_base",
        "//modules/localization/camera:camera_localization",
        "//modules/localization/rtk:rtk_localization",
    ],
)

cc_binary(
    name = "localization",
    srcs = [
        "main.cc",
    ],
    data = [
        "//modules/localization/conf:localization_adapter_manager_config",
    ],
    deps = [
        ":localization_app",
    ],
)

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

cpplint()
