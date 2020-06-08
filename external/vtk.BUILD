load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# FIXME(all): hide vtk version from end users
cc_library(
    name = "vtk",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lvtkCommonCore-8.2",
    ],
)
