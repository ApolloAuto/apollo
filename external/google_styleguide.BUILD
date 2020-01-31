package(default_visibility = ["//visibility:public"])

licenses(["notice"])

# We can't set name="cpplint" here because that's the directory name so the
# sandbox gets confused.  We'll give it a private name with a public alias.
py_binary(
    name = "cpplint_binary",
    srcs = ["cpplint/cpplint.py"],
    imports = ["cpplint"],
    main = "cpplint/cpplint.py",
    visibility = [],
)

alias(
    name = "cpplint",
    actual = ":cpplint_binary",
)

py_test(
    name = "cpplint_unittest",
    size = "small",
    srcs = ["cpplint/cpplint_unittest.py"],
    data = ["cpplint/cpplint_test_header.h"],
    deps = [
        ":cpplint_py",
    ],
)
