package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "adolc",
    includes = ["."],
    copts = [ "-fPIC"],
    linkopts = [
        "-L/usr/local/adolc/lib64 -ladolc",
    ],
)