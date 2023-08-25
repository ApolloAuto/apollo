load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "ndt_map_creator",
    srcs = ["ndt_map_creator.cc"],
    deps = [
        "//cyber",
        "//modules/localization/msf:apollo_localization_msf",
        "@boost",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_package()
cpplint()
