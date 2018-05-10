package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "local_integ",
    srcs = [
        "lib/liblocalization_msf.so",
    ],
    hdrs = glob([
        "include/*.h",
    ]),
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu",
        "-lz",
    ],
)
