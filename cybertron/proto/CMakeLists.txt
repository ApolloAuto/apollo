project(cybertron_proto)

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

set(PY_PROTOS)
foreach(FIL ${PROTO_FILES})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_PATH ${FIL} PATH)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    STRING(REGEX REPLACE "^(.*)\\.[^\\.]*$" "\\1" ABS_FIL_PRE ${ABS_FIL})
    set(ABS_FIL_PRE "${FIL_PATH}/${FIL_WE}")
    list(APPEND PY_PROTOS "${ABS_FIL_PRE}_pb2.py")

    add_custom_command(
        OUTPUT "${ABS_FIL_PRE}_pb2.py"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --python_out ${FIL_PATH} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL}
        DEPENDS ${ABS_FIL}
        COMMENT "Running python protocol buffer compiler on ${FIL}"
        VERBATIM )
endforeach()
set_source_files_properties(${PY_PROTOS} PROPERTIES GENERATED TRUE)
add_custom_target(py_proto ALL DEPENDS ${PY_PROTOS})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

set_source_files_properties(${PROTO_SRCS} PROPERTIES COMPILE_FLAGS -Wno-effc++)

add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} protobuf)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

file(REMOVE ${PROTO_SRCS})


install(FILES ${PROTO_HEADERS}
    DESTINATION include/cybertron/proto)

install(FILES ${PY_PROTOS} __init__.py
    DESTINATION lib/python/cybertron/proto)

install(TARGETS cybertron_proto LIBRARY DESTINATION lib)
