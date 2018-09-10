project(cybertron_proto)

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

set_source_files_properties(${PROTO_SRCS} PROPERTIES COMPILE_FLAGS -Wno-effc++)

add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS})

target_link_libraries(${PROJECT_NAME} protobuf)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

file(REMOVE ${PROTO_SRCS})

# install(FILES ${PROTO_HEADERS}
#     DESTINATION include/cybertron/proto
# )

install(TARGETS cybertron_proto LIBRARY DESTINATION lib)
