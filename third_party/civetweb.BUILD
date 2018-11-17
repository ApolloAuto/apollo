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
)

# The C++ wrapper for civetweb.
cc_library(
    name = "civetweb++",
    srcs = [
        "src/CivetServer.cpp",
    ],
    hdrs = [
        "include/CivetServer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":civetweb",
    ],
)
