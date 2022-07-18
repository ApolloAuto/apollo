load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

licenses(["notice"])

exports_files(["LICENSE.md"])

cc_library(
    name = "civetweb",
    srcs = [
        "src/civetweb.c",
    ],
    hdrs = [
        "include/civetweb.h",
        "src/handle_form.inl",
        "src/md5.inl",
    ],
    copts = [
        "-DUSE_WEBSOCKET",
    ],
    includes = [
        "include",
        "src",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    visibility = ["//visibility:public"],
    alwayslink = True,
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
    hdrs = [
        "include/CivetServer.h",
    ],
    includes = [
        "include",
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
