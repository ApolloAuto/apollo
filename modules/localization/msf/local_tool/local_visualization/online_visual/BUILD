load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_visualizer_compenont_lib",
    srcs = [
        "online_visualizer_component.cc",
    ],
    hdrs = [
        "online_visualizer_component.h",
    ],
    copts = [
        "-O2",
        "-DMODULE_NAME=\\\"localization_visualizer\\\"",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:geometry_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "@eigen",
        "@pcl",
    ],
)

cc_binary(
    name = "online_visualizer_compenont.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":online_visualizer_compenont_lib"],
)

cpplint()
