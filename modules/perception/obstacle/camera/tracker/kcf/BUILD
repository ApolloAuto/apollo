load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kcf",
    srcs = [
        "kcf_affinity_tracker.cc",
        "kcf_components.cc",
    ],
    hdrs = [
        "kcf_affinity_tracker.h",
        "kcf_components.h",
    ],
    deps = [
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
        "//modules/perception/obstacle/camera/tracker:util",
        "//modules/perception/obstacle/common",
        "@eigen",
        "@opencv2//:core",
    ],
)

cpplint()
