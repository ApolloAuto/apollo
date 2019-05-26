package(default_visibility = ["//visibility:public"])

licenses(
    # Note: For ADOL-C, EPL v1.0 license is chosen to be included.
    #       For ColPack, which is included by ADOL-C, is under LGPL v3.0.
    #       No derivative work or modification is made to the above two library.
    #       Only its generated binary is used.
    ["reciprocal"],
)

cc_library(
    name = "adolc",
    copts = ["-fPIC"],
    includes = ["."],
    linkopts = [
        "-L/usr/local/adolc/lib64 -ladolc",
        "-lgomp",
    ],
)
