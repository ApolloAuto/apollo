load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_generator",
    srcs = ["feature_generator.cc"],
    hdrs = ["feature_generator.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:instance_proto",
        "//modules/prediction/util:data_extraction",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":feature_generator",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
        "@opencv",
    ],
)

cpplint()
