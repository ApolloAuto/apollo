package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ffmpeg",
    hdrs = glob(["*"]),
    srcs = glob(["lib/*.so"]),
    includes = [
        "include",
    ],
    copts = [
        "-Iinclude",
    ],
    linkopts = [
        "-Llib",
        "-lavcodec",
        "-lavutil",
        "-lswresample",
    ]
)
