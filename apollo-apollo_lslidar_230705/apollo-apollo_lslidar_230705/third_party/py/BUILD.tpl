# Adapted with modifications from tensorflow/third_party/py/

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# config_setting(
#    name="python3",
#    flag_values = {"@rules_python//python:python_version": "PY3"}
# )

cc_library(
    name = "python_lib",
    deps = ["//_python3:_python3_lib"],
)

cc_library(
    name = "python_headers",
    deps = ["//_python3:_python3_headers"],
)
