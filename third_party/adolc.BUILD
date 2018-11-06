package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "adolc",
    includes = ["."],
    copts = [ "/usr/local/adolc/include"],
    linkopts = [
        "-L/usr/local/adolc/lib64 -ladolc",
    ],
)