load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "_cyber_init.so",
    deps = [
        ":py_init",
    ],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "py_init",
    srcs = [ "cyber_init_wrap.cc", ],
    hdrs = [ "py_init.h", ],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_node.so",
    deps = [
        ":py_node",
    ],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "py_node",
    srcs = [ "cyber_node_wrap.cc", ],
    hdrs = [ "py_node.h", ],
    deps = [
        "//cyber:cyber_core",
        "@python27",
    ],
)

cc_binary(
    name = "_cyber_record.so",
    deps = [
        ":py_record",
    ],
    linkshared = True,
    linkstatic = False,
)

cc_library(
    name = "py_record",
    srcs = [ "cyber_record_wrap.cc", ],
    hdrs = [ "py_record.h", ],
    deps = [
        "//cyber/record",
        "//cyber:cyber_core",
        "@python27",
    ],
)

cpplint()
