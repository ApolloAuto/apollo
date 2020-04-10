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
        "//modules/common/configs:config_gflags",
        "//modules/dreamview/proto:hmi_status_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:learning_data_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/prediction/util:data_extraction",
        "//modules/routing/proto:routing_proto",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "record_to_learning_data",
    srcs = ["record_to_learning_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":feature_generator",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
