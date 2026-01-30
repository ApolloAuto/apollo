load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

apollo_cc_library(
    name = "apollo_statistics",
    srcs = ["statistics.cc"],
    hdrs = ["statistics.h"],
    linkopts = ["-lbvar"],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/time:cyber_time",
    ],
)

apollo_package()

cpplint()
