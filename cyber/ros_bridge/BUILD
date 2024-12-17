load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_binary", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_cc_library(
    name = "converter_base",
    hdrs = [
        "converter_base/message_converter.h",
        "converter_base/converter_base.h", 
        "converter_base/convert_ros_quadruple.h",
        "converter_base/convert_ros_triple.h",
        "converter_base/convert_ros_double.h",
        "converter_base/convert_ros_single.h",
        "converter_base/convert_apollo_quadruple.h",
        "converter_base/convert_apollo_triple.h",
        "converter_base/convert_apollo_double.h",
        "converter_base/convert_apollo_single.h",
	    "converter_base/converter_interface.h",

        "common/bridge_argument.h",
        "common/ros_bridge_gflags.h",
        "common/utils.h",
        "common/macros.h",
    ],
    srcs = [
        "common/ros_bridge_gflags.cc",
        "common/bridge_argument.cc",
    ],
    deps = [
        "//cyber",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@com_github_gflags_gflags//:gflags",
        "@ros",
    ],
)

apollo_cc_binary(
    name = "ros_bridge",
    srcs = ["ros_bridge.cc"],
    linkopts = [
        "-ltcmalloc",
        "-lprofiler",
    ],
    deps = [
        ":converter_base",
        "//cyber/ros_bridge/proto:ros_bridge_conf_proto",
    ]
)

apollo_package()
cpplint()
