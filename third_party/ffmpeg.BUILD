package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ffmpeg",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib",
        "-lavcodec",
        "-lavutil",
        "-lswresample",
    ],
)
