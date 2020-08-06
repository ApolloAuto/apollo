load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_io",
    srcs = ["velodyne_utility.cc"],
    hdrs = glob(["*.h"]),
    deps = [
        "//cyber",
        "//modules/common/util:eigen_defs",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
    ],
)

cpplint()
