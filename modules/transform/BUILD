load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_transform",
    srcs = [
        "buffer.cc",
        "transform_broadcaster.cc",
    ],
    hdrs = [
        "buffer.h",
        "buffer_interface.h",
        "transform_broadcaster.h",
    ],
    deps = [
        "//cyber",
        "//third_party/tf2",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "@com_google_absl//:absl",
    ]
)

apollo_cc_test(
    name = "static_transform_component_test",
    size = "small",
    srcs = ["static_transform_component_test.cc"],
    deps = [
        ":DO_NOT_IMPORT_static_transform_component",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_component(
    name = "libstatic_transform_component.so",
    srcs = ["static_transform_component.cc"],
    hdrs = ["static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()

cpplint()
