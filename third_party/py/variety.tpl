package(default_visibility=["//visibility:public"])

cc_library(
    name="%{VARIETY_NAME}_lib",
    linkopts = ["-l%{PYTHON_SO_NAME}"],
)

cc_library(
    name="%{VARIETY_NAME}_headers",
    hdrs=[":%{VARIETY_NAME}_include"],
    includes=["%{VARIETY_NAME}_include"],
)

%{PYTHON_INCLUDE_GENRULE}
