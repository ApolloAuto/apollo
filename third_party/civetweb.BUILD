licenses(["notice"])

exports_files(["LICENSE.md"])

cc_library(
    name = "civetweb",
    visibility = ["//visibility:public"],
    hdrs = [
        "include/civetweb.h",
        "src/md5.inl",
        "src/handle_form.inl",
    ],
    srcs = [
        "src/civetweb.c",
    ],
    includes = [
        "include",
        "src",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
)

# The C++ wrapper for civetweb.
cc_library(
    name = "civetweb++",
    visibility = ["//visibility:public"],
    hdrs = [
        "include/CivetServer.h",
    ],
    srcs = [
        "src/CivetServer.cpp",
    ],
    deps = [
        ":civetweb",
    ],
)
