load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "msg_adapter_files",
    srcs = glob([
        "dag/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libmsg_adapter_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":msg_adapter_component_lib"],
)

cc_library(
    name = "msg_adapter_component_lib",
    srcs = ["msg_adapter_component.cc"],
    hdrs = ["msg_adapter_component.h"],
    deps = [
        ":msg_converter",
        "//cyber",
        "//modules/perception/msg_adapter/common:msg_adapter_gflags",
        "//modules/perception/msg_adapter/convert",
    ],
    alwayslink = True,
)

cc_library(
    name = "msg_converter",
    hdrs = ["msg_converter.h"],
    deps = [
        "//cyber",
    ]
)

apollo_package()
cpplint()
