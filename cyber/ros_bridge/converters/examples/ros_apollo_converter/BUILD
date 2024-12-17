load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

# import ros repository is new feature of cyber 2.0
# depend on the env configuration provided by the apollo tool
# currently under development
apollo_plugin(
    name = "libros_apollo_converter_plugin.so",
    srcs = ["ros_apollo_converter.cc"],
    hdrs = ["ros_apollo_converter.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge:converter_base",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@ros"
    ],
)

apollo_package()

cpplint()
