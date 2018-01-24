load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_base",
    srcs = [
        "localization_base.cc",
    ],
    hdrs = [
        "localization_base.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/status",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
        "@ros//:ros_common",
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
    ] + select({
        "//tools/platforms:x86_64": [
            "//modules/localization/msf:msf_localization",
            "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        ],
        "//conditions:default": [
        ],
    }) + [
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/localization/proto:measure_proto",
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
        ":localization_lib",
    ],
)

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

cpplint()
