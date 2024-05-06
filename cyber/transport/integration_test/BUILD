load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test")

apollo_cc_test(
    name = "hybrid_transceiver_test",
    size = "small",
    srcs = ["hybrid_transceiver_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "intra_transceiver_test",
    size = "small",
    srcs = ["intra_transceiver_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rtps_transceiver_test",
    size = "small",
    srcs = ["rtps_transceiver_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "shm_transceiver_test",
    size = "small",
    srcs = ["shm_transceiver_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_package()
cpplint()
