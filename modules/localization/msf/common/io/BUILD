load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_io",
    srcs = ["velodyne_utility.cc"],
    hdrs = glob(["*.h"]),
    deps = [
        "//cyber",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
