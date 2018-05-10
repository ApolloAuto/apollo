licenses(["notice"])

cc_library(
    name = "curlpp",
    srcs = [
        "src/curlpp/Easy.cpp",
        "src/curlpp/Exception.cpp",
        "src/curlpp/Form.cpp",
        "src/curlpp/OptionBase.cpp",
        "src/curlpp/Options.cpp",
        "src/curlpp/cURLpp.cpp",
        "src/curlpp/internal/CurlHandle.cpp",
        "src/curlpp/internal/OptionList.cpp",
        "src/curlpp/internal/OptionSetter.cpp",
        "src/curlpp/internal/SList.cpp",
    ],
    hdrs = [
        "include/curlpp/Easy.hpp",
        "include/curlpp/Exception.hpp",
        "include/curlpp/Options.hpp",
        "include/curlpp/cURLpp.hpp",
    ],
    includes = [
        "include",
        "include/curlpp",
    ],
    linkopts = [
        "-lcurl",
    ],
    visibility = ["//visibility:public"],
)
