load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "manual_traffic_light",
    srcs = ["manual_traffic_light.cc"],
    copts = ["-DMODULE_NAME=\\\"manual_traffic_light\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/map/hdmap:hdmap_util",
    ],
)

cc_binary(
    name = "libmanual_traffic_light_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":manual_traffic_light"],
)

cpplint()
