package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "pytorch",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
        "-Iinclude/torch",
        "-Iinclude/torch/csrc/api/include/torch",
    ],
    includes = [
        "include",
        "include/torch/csrc/api/include",
        "include/torch/csrc/api/include/torch",
    ],
    linkopts = [
        "-Llib",
    ],
    linkstatic = False,
    deps = [
        "@python27",
    ],
)
