load("@rules_python//python:defs.bzl", "py_binary", "py_test")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

# We can't set name="cpplint" here because that's the directory name so the
# sandbox gets confused.  We'll give it a private name with a public alias.
py_binary(
    name = "cpplint_binary",
    srcs = ["cpplint.py"],
    imports = ["cpplint"],
    main = "cpplint.py",
    visibility = [],
)

alias(
    name = "cpplint",
    actual = ":cpplint_binary",
)

py_test(
    name = "cpplint_unittest",
    size = "small",
    srcs = ["cpplint_unittest.py"],
    data = ["cpplint_test_header.h"],
    deps = [
        ":cpplint_py",
    ],
)
