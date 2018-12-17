load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "manual_traffic_light",
    srcs = ["manual_traffic_light.cc"],
    copts = ["-DMODULE_NAME=\\\"manual_traffic_light\\\""],
    data = ["//modules/map:map_data"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:color",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_binary(
    name = "libmanual_traffic_light_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":manual_traffic_light"],
)

cpplint()
