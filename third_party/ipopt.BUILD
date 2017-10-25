package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ipopt",
    includes = ["."],
    linkopts = [
        "-L/usr/local/ipopt/lib -lipopt",
        "-L/usr/local/ipopt/lib -lcoinmumps",
        "-lblas",
        "-llapack",
        "-ldl",
        "-lgfortran",
    ],
)