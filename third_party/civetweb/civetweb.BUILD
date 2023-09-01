load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

licenses(["notice"])

exports_files(["LICENSE.md"])

cc_library(
    name = "civetweb",
    srcs = [
        "src/civetweb.c",
    ],
    #hdrs = glob([
    #    "include/*.h",
    #    "src/*.inl",
    #    #"src/md5.inl",
    #]),
    copts = [
        "-DUSE_WEBSOCKET",
        "-DNO_SSL",
    ],
    includes = [
        "include",
        "src",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "h_headers",
        "inl_headers"
    ],
    visibility = ["//visibility:public"],
    alwayslink = True,
)

cc_library(
    name = "h_headers",
    hdrs = glob([
        "include/*.h",
    ]),
    strip_include_prefix = "include",
)

cc_library(
    name = "inl_headers",
    hdrs = glob([
        "src/*.inl",
    ]),
    strip_include_prefix = "src",
)

cc_binary(
    name = "libcivetweb++.so",
    srcs = [
        "src/CivetServer.cpp",
        "include/CivetServer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":civetweb",
    ],
    linkshared = True,
    linkstatic = True,
)

# The C++ wrapper for civetweb.
cc_library(
    name = "civetweb++",
    srcs = [
        "libcivetweb++.so",
    ],
    #hdrs = glob([
    #    "include/*.h",
    #]),
    includes = [
        "include",
    ],
    deps = [
        "h_headers",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "civetweb_hdrs",
    srcs = glob([
        "include/*.h",
    ]),
    visibility = ["//visibility:public"],
)
filegroup(
    name = "civetweb_libs",
    srcs = [
        ":libcivetweb++.so",
    ],
    visibility = ["//visibility:public"],
)
