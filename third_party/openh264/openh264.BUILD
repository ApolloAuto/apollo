load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "openh264",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        # "-L/usr/local/lib",
        "-L/opt/apollo/sysroot/lib",
        "-lopenh264",
    ],
    linkstatic = False,
)
