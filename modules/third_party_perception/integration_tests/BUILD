load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_test_base",
    srcs = [
        "third_party_perception_test_base.cc",
    ],
    hdrs = [
        "third_party_perception_test_base.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "//modules/third_party_perception:third_party_perception_lib",
        "//modules/third_party_perception/common:third_party_perception_gflags",
        "//modules/third_party_perception/proto:third_party_perception_proto",
        "@gtest//:main",
    ],
)

cpplint()
