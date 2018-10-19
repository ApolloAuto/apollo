package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "python27",
    srcs = glob([
        "lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so",
    ]),
    hdrs = glob([
        "include/python2.7/*.h",
    ]),
    includes = ["include/python2.7"],
)
