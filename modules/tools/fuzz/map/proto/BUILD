package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "hdmap_fuzz_proto",
    deps = [
        ":hdmap_fuzz_proto_lib",
    ],
)

proto_library(
    name = "hdmap_fuzz_proto_lib",
    srcs = [
        "hdmap_fuzz.proto",
    ],
    deps = [
        "//modules/map/proto:map_proto_lib",
    ],
)