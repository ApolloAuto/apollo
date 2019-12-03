load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "roi_service_proto",
    deps = [
        ":roi_service_proto_lib",
    ],
)

proto_library(
    name = "roi_service_proto_lib",
    srcs = ["roi_service.proto"],
)

cpplint()
