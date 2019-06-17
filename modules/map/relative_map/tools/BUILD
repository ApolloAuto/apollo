load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "navigation_dummy",
    srcs = ["navigation_dummy.cc"],
    copts = ["-DMODULE_NAME=\\\"map\\\""],
    deps = [
        "//cyber",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/map/relative_map:relative_map_lib",
    ],
)

cc_binary(
    name = "navigator",
    srcs = ["navigator.cc"],
    copts = ["-DMODULE_NAME=\\\"map\\\""],
    deps = [
        "//cyber",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/map/relative_map:relative_map_lib",
        "//modules/map/relative_map/proto:navigator_config_proto",
        "//third_party/json",
    ],
)

cpplint()
