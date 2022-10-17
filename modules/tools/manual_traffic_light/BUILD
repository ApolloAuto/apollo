load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "tools/lib",
    data_dest = "tools/manual_traffic_light",
    data = [":conf_files"],
    targets = [":libmanual_traffic_light_component.so"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "conf_files",
    srcs = [
        "manual_traffic_light.conf",
        "manual_traffic_light.dag",
        "manual_traffic_light.launch",
    ],
)

cc_library(
    name = "manual_traffic_light",
    srcs = ["manual_traffic_light.cc"],
    copts = ["-DMODULE_NAME=\\\"manual_traffic_light\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
    ],
)

cc_binary(
    name = "libmanual_traffic_light_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":manual_traffic_light"],
)

cpplint()
