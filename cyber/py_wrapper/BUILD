load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "_cyber_init.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_init",
    ],
)

cc_library(
    name = "py_init",
    srcs = ["cyber_init_wrap.cc"],
    hdrs = ["py_init.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_node.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_node",
    ],
)

cc_library(
    name = "py_node",
    srcs = ["cyber_node_wrap.cc"],
    hdrs = ["py_node.h"],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_record.so",
    linkshared = True,
    deps = [
        ":py_record",
    ],
)

cc_library(
    name = "py_record",
    srcs = ["cyber_record_wrap.cc"],
    hdrs = ["py_record.h"],
    deps = [
        "//cyber/message:py_message",
        "//cyber/record",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_topology_manager.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":py_topology_manager",
    ],
)

cc_library(
    name = "py_topology_manager",
    srcs = ["cyber_topology_manager_wrap.cc"],
    hdrs = ["py_topology_manager.h"],
    deps = [
        "//cyber/service_discovery:topology_manager",
        "//cyber:cyber_core",
        "@python27",
    ],
)
cpplint()
