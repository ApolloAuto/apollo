# WebSocket++, also referred to as Websocketpp, is a header only C++ library
# that implements RFC6455 The WebSocket Protocol. It allows integrating WebSocket
# client and server functionality into C++ programs. It uses interchangeable
# network transport modules including one based on raw char buffers, one based
# on C++ iostreams, and one based on Asio (either via Boost or standalone).
# End users can write additional transport policies to support other networking
# or event libraries as needed.

licenses(["notice"])

cc_library(
    name = "websocketpp",
    visibility = ["//visibility:public"],
    hdrs = glob([
        "websocketpp/**/*.hpp",
    ]),
    includes = [
        "websocketpp",
        ".",
    ],
    linkopts = [
        "-pthread",
        "-lboost_system",
    ],
)
