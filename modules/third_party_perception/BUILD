load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_lib",
    srcs = ["third_party_perception.cc"],
    hdrs = ["third_party_perception.h"],
    deps = [
        ":third_party_perception_conversion",
        ":third_party_perception_filter",
        ":third_party_perception_fusion",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
    ],
)

cc_library(
    name = "third_party_perception_conversion",
    srcs = ["conversion.cc"],
    hdrs = ["conversion.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_fusion",
    srcs = ["fusion.cc"],
    hdrs = ["fusion.h"],
    deps = [
        "//modules/common/math:geometry",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_filter",
    srcs = ["filter.cc"],
    hdrs = ["filter.h"],
    deps = [
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_component_lib",
    srcs = ["third_party_perception_component.cc"],
    hdrs = ["third_party_perception_component.h"],
    copts = ['-DMODULE_NAME=\\"third_party_perception\\"'],
    deps = [
        ":third_party_perception_lib",
    ],
)

cc_binary(
    name = "libthird_party_perception_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":third_party_perception_component_lib"],
)

cpplint()
