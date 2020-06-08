package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ffmpeg",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavcodec",
        "-lavutil",
        "-lswresample",
    ],
)
