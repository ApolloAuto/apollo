load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLATFORM_COPTS=["-D__AARCH64__=1"]

cc_library(
    name = "i_ground",
    srcs = ["i_ground.cc"],
    hdrs = ["i_ground.h"],
    alwayslink = True,
    deps = [
        ":i_struct_s",
        ":i_util",
        "//modules/perception/common/algorithm/i_lib/algorithm:i_sort",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/da:i_ransac",
        "//modules/perception/common/algorithm/i_lib/geometry:i_plane",
    ],
)

cc_library(
    name = "i_struct_s",
    hdrs = ["i_struct_s.h"],
    alwayslink = True,
    deps = select(
        {
            "@platforms//cpu:x86_64": [],
            "@platforms//cpu:aarch64": [
                "//third_party/intrinsics_translation",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    # copts = select(
    #     {
    #         "@platforms//cpu:x86_64": [],
    #         "@platforms//cpu:aarch64": PLATFORM_COPTS,
    #     },
    #     no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    # ),
)

cc_library(
    name = "i_util",
    hdrs = ["i_util.h"],
    alwayslink = True,
)

apollo_package()
cpplint()

