#licenses(["notice"])

#package(default_visibility = ["//visibility:public"])

#cc_library(
#    name = "yaml",
#    includes = ["."],
#    linkopts = [
#        "-lyaml-cpp",
#    ],
#)
cc_library(
    name = "yaml",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/yaml-cpp/*.h",
        "src/*.h",
    ]),
    includes = [
        "include",
        "src",
    ],
    visibility = ["//visibility:public"],
)

