load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_util",
    srcs = ["third_party_perception_util.cc"],
    hdrs = ["third_party_perception_util.h"],
    deps = [
        ":third_party_perception_gflags",
        "//modules/common:log",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_gflags",
    srcs = [
        "third_party_perception_gflags.cc",
    ],
    hdrs = [
        "third_party_perception_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
