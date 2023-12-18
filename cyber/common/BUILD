load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_common",
    srcs = [
        "global_data.cc",
        "file.cc",
    ],
    data = [
        "//cyber:cyber_conf",
    ],
    hdrs = [
        "environment.h",
        "file.h",
        "global_data.h",
        "log.h",
        "macros.h",
        "time_conversion.h",
        "types.h",
        "util.h",
    ],
    deps = [
        "//cyber:cyber_binary",
        "//cyber/base:cyber_base",
        "//cyber/proto:cyber_conf_cc_proto",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
        "@com_github_google_glog//:glog",
    ],
)

apollo_cc_test(
    name = "file_test",
    size = "small",
    srcs = ["file_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "log_test",
    size = "small",
    srcs = ["log_test.cc"],
    linkstatic = True,
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "environment_test",
    size = "small",
    srcs = ["environment_test.cc"],
    deps = [
        ":cyber_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "macros_test",
    size = "small",
    srcs = ["macros_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
