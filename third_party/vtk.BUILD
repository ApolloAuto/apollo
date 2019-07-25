licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# This assumes you have vtk pre-installed in your system.
cc_library(
    name = "vtk",
    copts = [
        "-Wno-deprecated",
    ],
    includes = ["."],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu/",
        "-lvtkCommonCore-7.1",
    ],
)
