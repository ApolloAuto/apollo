load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "routing",
    srcs = ["routing.cc"],
    hdrs = ["routing.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "routing_component_lib",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":routing",
        "//cyber",
    ],
    alwayslink = True,
)

cc_binary(
    name = "librouting_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":routing_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data_dest = "routing",
    library_dest = "routing/lib",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":routing.BUILD",
    ],
    targets = [
        ":librouting_component.so",
    ],
    deps = [
        ":pb_routing",
        ":pb_hdrs",
        "//modules/routing/topo_creator:install",
        "//modules/routing/tools/routing_tester:install",
        "//modules/routing/tools:install",
        "//modules/routing/proto:py_pb_routing"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "routing/include",
    data = [
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",

    ],
)

install_files(
    name = "pb_routing",
    dest = "routing",
    files = [
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_routing_src",
        ":install_routing_hdrs"
    ],
)

install_src_files(
    name = "install_routing_src",
    src_dir = ["."],
    dest = "routing/src",
    filter = "*",
)

install_src_files(
    name = "install_routing_hdrs",
    src_dir = ["."],
    dest = "routing/include",
    filter = "*.h",
)
cpplint()
