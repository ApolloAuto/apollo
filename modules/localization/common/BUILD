load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")
# load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_compensator",
    srcs = [
        "gnss_compensator.cc",
    ],
    hdrs = [
        "gnss_compensator.h",
    ],
    deps = [
        "//cyber",
        ":localization_gflags",
    ],
)

cc_library(
    name = "localization_gflags",
    srcs = [
        "liblocalization_gflags.so",
    ],
    hdrs = [
        "localization_gflags.h",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "liblocalization_gflags.so",
    srcs = ["localization_gflags.cc", "localization_gflags.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

# install(
#     name = "install",
#     library_dest = "localization/lib",
#     targets = [
#         ":liblocalization_gflags.so",
#     ],
# )

apollo_package()
cpplint()
