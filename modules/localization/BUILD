load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_base",
    hdrs = [
        "localization_base.h",
    ],
)

cc_library(
    name = "localization_lib",
    srcs = [
        "localization.cc",
    ],
    hdrs = [
        "localization.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:apollo_app",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/localization:localization_base",
        "//modules/localization/rtk:rtk_localization",
        "//modules/localization/msf:msf_localization",
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
        ":localization_lib",
    ],
)

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

cpplint()
