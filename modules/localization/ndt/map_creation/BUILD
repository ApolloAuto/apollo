load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_map_creator",
    srcs = ["ndt_map_creator.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:system_utility",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:ndt_map_pool",
        "@boost",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

install(
    name = "install",
    runtime_dest = "localization/bin",
    targets = [
        ":ndt_map_creator",
    ],
)

cpplint()
