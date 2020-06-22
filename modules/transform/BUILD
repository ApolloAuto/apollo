load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tf2_buffer_lib",
    srcs = ["buffer.cc"],
    hdrs = [
        "buffer.h",
        "buffer_interface.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "@com_google_absl//absl/strings",
        "@tf2",
    ],
)

cc_library(
    name = "transform_broadcaster_lib",
    srcs = ["transform_broadcaster.cc"],
    hdrs = ["transform_broadcaster.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/transform/proto:transform_cc_proto",
    ],
)

cc_library(
    name = "static_transform_component_lib",
    srcs = ["static_transform_component.cc"],
    hdrs = ["static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_test(
    name = "static_transform_component_test",
    size = "small",
    srcs = ["static_transform_component_test.cc"],
    deps = [
        ":static_transform_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libstatic_transform_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":static_transform_component_lib"],
)

cpplint()
