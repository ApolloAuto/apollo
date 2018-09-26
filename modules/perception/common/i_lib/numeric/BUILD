load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "numeric",
    hdrs = [
        "i_eig.h",
        "i_householder.h",
        "i_lu.h",
        "i_poly.h",
        "i_qr.h",
    ],
    deps = [
        "//modules/perception/common/i_lib/core",
    ],
)

cpplint()
