package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "third_party_perception_fuzz_proto",
    deps = [
        ":third_party_perception_fuzz_proto_lib",
    ],
)

proto_library(
    name = "third_party_perception_fuzz_proto_lib",
    srcs = [
        "third_party_perception_fuzz.proto",
    ],
    deps = [
       "//modules/drivers/proto:sensor_proto_lib",
       "//modules/canbus/proto:canbus_proto_lib",
       "//modules/localization/proto:localization_proto_lib",
    ],
)