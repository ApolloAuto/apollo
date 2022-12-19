load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GUARDIAN_COPTS = ['-DMODULE_NAME=\\"guardian\\"']

cc_library(
    name = "guardian_component_lib",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = GUARDIAN_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
        "//modules/guardian/proto:guardian_conf_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libguardian_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":guardian_component_lib",
    ],
)

install(
    name = "install",
    library_dest = "guardian/lib",
    data_dest = "guardian",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":guardian.BUILD",
    ],
    targets = [
        ":libguardian_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/guardian/proto:py_pb_guardian"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "guardian/include",
    data = [
        "//modules/guardian/proto:guardian_conf_cc_proto",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_guardian_src",
        ":install_guardian_hdrs"
    ],
)

install_src_files(
    name = "install_guardian_src",
    src_dir = ["."],
    dest = "guardian/src",
    filter = "*",
)

install_src_files(
    name = "install_guardian_hdrs",
    src_dir = ["."],
    dest = "guardian/include",
    filter = "*.h",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
