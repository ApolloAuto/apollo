package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "paddlepaddle",
    includes = ["."],
    linkopts = [
        "-L/usr/local/apollo/paddlepaddle/lib",
        "-lpaddle_fluid",
],
)
