load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_common",
    srcs = [
        "file.cc",
        "global_data.cc",
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
    data = [
        "//cyber:cyber_conf",
    ],
    deps = [
        "//cyber:cyber_binary",
        "//cyber/base:cyber_base",
        "//cyber/proto:cyber_conf_cc_proto",
        "//cyber/proto:transport_conf_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
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
