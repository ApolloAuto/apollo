load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

install(
    name = "install",
    runtime_dest = "map/bin",
    targets = [
        ":navigation_dummy",
        ":navigator",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "navigation_dummy",
    srcs = ["navigation_dummy.cc"],
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/map/relative_map:relative_map_lib",
    ],
)

cc_binary(
    name = "navigator",
    srcs = ["navigator.cc"],
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/map/relative_map:relative_map_lib",
        "//modules/map/relative_map/proto:navigator_config_cc_proto",
        "@com_github_nlohmann_json//:json",
    ],
)

cpplint()
