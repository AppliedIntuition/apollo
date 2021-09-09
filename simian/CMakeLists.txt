cmake_minimum_required(VERSION 2.8)

project (simian_customer_name C CXX)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)

# Eigen3
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# geographiclib - Needed for coordinate tranformations
# Must use our own FindGeographicLib.cmake to support /usr/lib/x86_64-linux-gnu/
# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/third_party/")
find_package (GeographicLib REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})
add_definitions (${GeographicLib_DEFINITIONS})

# libprotobuf
include(FindProtobuf)
find_package(Protobuf CONFIG REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
# gRPC is required for using the Simian Map Client.
# Threads is required by gRPC.
find_package(Threads REQUIRED)
find_package(gRPC CONFIG REQUIRED)
set(PROTOBUF_PROTOC_EXECUTABLE $<TARGET_FILE:protobuf::protoc>)
set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

add_definitions (-Wall -Werror -g -DHAVE_PTHREAD -Wno-error=sign-compare -Wno-unused-value)
# The following warning is created by generated protobuf code.
add_definitions (-Wno-deprecated-declarations)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -O3")

include_directories (
  .
  ${CMAKE_BINARY_DIR}
)

set(PROTOBUF_IMPORT_DIRS ${PROJECT_SOURCE_DIR})

# Protobuf generation
file(GLOB_RECURSE APPLIED_PROTOS RELATIVE ${PROJECT_SOURCE_DIR} "applied/*.proto")
# We handle this proto separately below.
set(PUBLIC_MAP_SERVICE_PROTO_FILEPATH "applied/simian/public/proto/public_map_service.proto")
list(REMOVE_ITEM APPLIED_PROTOS ${PUBLIC_MAP_SERVICE_PROTO_FILEPATH})

set(PROTO_SRCS)
set(PROTO_HDRS)
# Generate proto code.
foreach(FIL ${APPLIED_PROTOS})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)
  get_filename_component(DIR ${FIL} DIRECTORY)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out ${CMAKE_CURRENT_BINARY_DIR} -I ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/${FIL}
    DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM )

endforeach()

# Generate gRPC code.
set(GRPC_PROTOS ${PUBLIC_MAP_SERVICE_PROTO_FILEPATH})
foreach(FIL IN LISTS GRPC_PROTOS)
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)
  get_filename_component(DIR ${FIL} DIRECTORY)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h")

  add_custom_command(
          OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
                 "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
                 "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc"
                 "${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h"
          COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
          ARGS --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
               --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
               --cpp_out ${CMAKE_CURRENT_BINARY_DIR}
               -I ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/${FIL}
          DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
          WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
          COMMENT "Running C++ protocol buffer compiler with gRPC plugin on ${FIL}"
          VERBATIM )
endforeach()

SET_SOURCE_FILES_PROPERTIES(${PROTO_SRCS} PROPERTIES GENERATED TRUE)
# End protobuf generation

file (GLOB_RECURSE APPLIED_CXX "applied/*.cc")

add_library (applied_cxx SHARED ${APPLIED_CXX} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries (applied_cxx gRPC::grpc++)

add_library (
  customer_interface SHARED
  applied/customer_interface_glue.cc
  applied/customer_stack_base.cc
  interface/customer_stack.cc
  interface/log_reader.cc
  )
target_link_libraries (customer_interface applied_cxx)
target_link_libraries (customer_interface protobuf::libprotobuf)
target_link_libraries (customer_interface gRPC::grpc++)
target_link_libraries (customer_interface ${GeographicLib_LIBRARIES})

set_target_properties (customer_interface PROPERTIES OUTPUT_NAME "customer_interface")
set_target_properties (customer_interface PROPERTIES PREFIX "")
