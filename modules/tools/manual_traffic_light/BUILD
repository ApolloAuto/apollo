load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "manual_traffic_light",
    srcs = ["manual_traffic_light.cc"],
    copts = ["-DMODULE_NAME=\\\"manual_traffic_light\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:color",
        "//modules/common/util:message_util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
    ],
)

cc_binary(
    name = "libmanual_traffic_light_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":manual_traffic_light"],
)

cpplint()
