# Adapted from https://github.com/mzhaom/trunk/blob/master/third_party/gtest/BUILD
licenses(["notice"])
exports_files(["googletest/LICENSE"])

cc_library(
    name = "gtest",
    testonly = 1,
    srcs = glob([
        "googletest/include/gtest/internal/**/*.h",
    ]) + [
        "googletest/src/gtest-internal-inl.h",
        "googletest/src/gtest-death-test.cc",
        "googletest/src/gtest-filepath.cc",
        "googletest/src/gtest-port.cc",
        "googletest/src/gtest-printers.cc",
        "googletest/src/gtest-test-part.cc",
        "googletest/src/gtest-typed-test.cc",
        "googletest/src/gtest.cc",
    ],
    hdrs = [
        "googletest/include/gtest/gtest.h",
        "googletest/include/gtest/gtest-death-test.h",
        "googletest/include/gtest/gtest-message.h",
        "googletest/include/gtest/gtest-param-test.h",
        "googletest/include/gtest/gtest-printers.h",
        "googletest/include/gtest/gtest-spi.h",
        "googletest/include/gtest/gtest-test-part.h",
        "googletest/include/gtest/gtest-typed-test.h",
        "googletest/include/gtest/gtest_pred_impl.h",
    ],
    copts = [
        "-Iexternal/gtest/googletest",
    ],
    includes = [
        "googletest/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gtest_prod",
    ],
)

cc_library(
    name = "gtest_main",
    testonly = 1,
    srcs = [
        "googletest/src/gtest_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gtest",
    ],
)

cc_library(
    name = "gtest_prod",
    hdrs = [
        "googletest/include/gtest/gtest_prod.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "googlemock",
    testonly = 1,
    srcs = glob([
        "googlemock/include/gmock/internal/**/*.h",
    ]) + [
        "googlemock/src/gmock-cardinalities.cc",
        "googlemock/src/gmock.cc",
        "googlemock/src/gmock-internal-utils.cc",
        "googlemock/src/gmock-matchers.cc",
        "googlemock/src/gmock-spec-builders.cc",
    ],
    hdrs = [
        "googlemock/include/gmock/gmock.h",
        "googlemock/include/gmock/gmock-actions.h",
        "googlemock/include/gmock/gmock-cardinalities.h",
        "googlemock/include/gmock/gmock-generated-actions.h",
        "googlemock/include/gmock/gmock-generated-function-mockers.h",
        "googlemock/include/gmock/gmock-generated-matchers.h",
        "googlemock/include/gmock/gmock-generated-nice-strict.h",
        "googlemock/include/gmock/gmock-matchers.h",
        "googlemock/include/gmock/gmock-more-actions.h",
        "googlemock/include/gmock/gmock-more-matchers.h",
        "googlemock/include/gmock/gmock-spec-builders.h",
    ],
    includes = [
        "googlemock/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gtest",
    ],
)

cc_library(
    name = "gmock_main",
    srcs = [
        "googlemock/src/gmock_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gmock",
    ],
)
