load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "gnss_compensator",
    srcs = [
        "gnss_compensator.cc",
    ],
    hdrs = [
        "gnss_compensator.h",
    ],
    deps = [
        ":localization_gflags",
        "//cyber",
    ],
)

apollo_cc_library(
    name = "localization_gflags",
    srcs = ["localization_gflags.cc"],
    hdrs = [
        "localization_gflags.h",
    ],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_package()

cpplint()
