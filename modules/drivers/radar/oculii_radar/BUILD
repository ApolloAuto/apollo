load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_library(
    name = "oculii_radar_parser",
    srcs = ["parser/oculii_radar_udp_parser.cc"],
    hdrs = [
        "parser/oculii_radar_udp_parser.h",
        "common/const_val.h",
        "common/type_def.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:oculii_radar_proto",
        "@eigen",
    ],
)

apollo_component(
    name = "liboculii_radar_component.so",
    srcs = ["oculii_radar_component.cc"],
    hdrs = ["oculii_radar_component.h"],
    deps = [
        ":oculii_radar_parser",
        "//modules/drivers/radar/oculii_radar/proto:oculii_radar_conf_proto",
        "//modules/common/util:util_tool",
    ]
)


apollo_package()
cpplint()
