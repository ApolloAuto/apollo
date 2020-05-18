package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ffmpeg",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/usr/local/ffmpeg4/lib",
        "-lavcodec",
        "-lavutil",
        "-lswresample",
    ],
)
