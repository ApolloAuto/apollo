load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ssl",
    hdrs = glob([
        "include/openssl/**/*",
    ]),
    srcs = glob([
      "lib/*"
    ]),
    includes = ["include"],
    linkopts = [
        "-lssl",
    ],
    strip_include_prefix = "include",
)
