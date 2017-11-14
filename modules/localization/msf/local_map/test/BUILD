load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "localization_msf_local_map_test",
    size = "medium",
    timeout = "short",
    srcs = glob(["base_map/*.cc",
                "lossless_map/*.cc",
                "lossy_map/*.cc"]),
    data = [
        ":testdata",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossless_map:localization_msf_lossless_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "@glog//:glog",
        "@gtest//:main"
    ],
)

cpplint()
