package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ipopt",
    includes = ["."],
    copts = [ "-fPIC"],
    linkopts = [
        "-L/usr/local/ipopt/lib -lipopt",
        "-L/usr/local/ipopt/lib -lcoinmumps",
        "-lblas",
        "-llapack",
        "-ldl",
        "-L/usr/lib/gcc/x86_64-linux-gnu/7/ -lgfortran",
    ],
)
