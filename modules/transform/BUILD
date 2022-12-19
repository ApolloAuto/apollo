load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libbuffer.so",
    srcs = [
        "buffer.cc",
        "buffer.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":buffer_interface",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//third_party/tf2",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "buffer",
    srcs = ["libbuffer.so"],
    hdrs = [
        "buffer.h",
        "buffer_interface.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//third_party/tf2",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "buffer_interface",
    hdrs = ["buffer_interface.h"],
    deps = [
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
    ],
    visibility = ["//visibility:private"],
)

cc_binary(
    name = "libtransform_broadcaster.so",
    srcs = [
        "transform_broadcaster.cc",
        "transform_broadcaster.h"
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "transform_broadcaster",
    srcs = ["libtransform_broadcaster.so"],
    hdrs = ["transform_broadcaster.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "static_transform_component_lib",
    srcs = ["static_transform_component.cc"],
    hdrs = ["static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
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
    linkstatic = True,
    deps = [":static_transform_component_lib"],
)

install(
    name = "install",
    data_dest = "transform",
    library_dest = "transform/lib",
    data = [
        ":runtime_data",
        "cyberfile.xml",
        "transform.BUILD",
    ],
    targets = [
        ":libstatic_transform_component.so",
        ":libbuffer.so",
        ":libtransform_broadcaster.so",
    ],
    deps = [
        "//modules/transform/proto:py_pb_tf"
    ]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_transform_src",
        ":install_transform_hdrs"
    ],
)

install_src_files(
    name = "install_transform_src",
    src_dir = ["."],
    dest = "transform/src",
    filter = "*",
)

install_src_files(
    name = "install_transform_hdrs",
    src_dir = ["."],
    dest = "transform/include",
    filter = "*.h",
)

cpplint()
