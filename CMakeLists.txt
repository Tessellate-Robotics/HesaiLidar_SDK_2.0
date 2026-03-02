cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
PROJECT(Template)
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  9)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)
# add_definitions(-DJT128_256)

# 设置生成平台为 x64
if(MSVC)
add_compile_options("/source-charset:utf-8")
set(CMAKE_GENERATOR_PLATFORM x64)
set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")
set(OPENSSL_INCLUDE_DIR ${OPENSSL_ROOT_DIR}/include)
endif()


include_directories("${PROJECT_BINARY_DIR}")
find_package(OpenSSL REQUIRED)
add_definitions(-DQT_MESSAGELOGCONTEXT)
add_subdirectory(libhesai)

include_directories(
  libhesai
  libhesai/Lidar
  libhesai/UdpParser
  libhesai/UdpParser/include
  libhesai/UdpParser/src
  libhesai/UdpProtocol
  libhesai/Source/include
  libhesai/Container/include
  libhesai/Container/src
  libhesai/UdpParserGpu
  libhesai/UdpParserGpu/include
  libhesai/UdpParserGpu/src
  libhesai/PtcClient/include
  libhesai/SerialClient/include
  libhesai/PtcParser
  libhesai/PtcParser/src
  libhesai/PtcParser/include
  libhesai/Logger/include
  libhesai/include
  driver
  third_party
  ${OPENSSL_INCLUDE_DIR}
)
link_directories(./libhesai/PtcClient/lib)


target_link_libraries(sample
  source_lib
  container_lib
  ptcClient_lib
  serialClient_lib
  ptcParser_lib
  log_lib
  ${OPENSSL_LIBRARIES}
)



if(BUILD_LOG_LIB_SHARED)
	target_compile_definitions(hesai_sdk_lib PUBLIC BUILD_LOG_LIB_SHARED=1)
endif(BUILD_LOG_LIB_SHARED)

if(DISENABLE_TEST_CC)

else()
	add_executable(sample
		test/test.cc
		)

	target_link_libraries(sample
		hesai_sdk_lib
		)

	add_executable(multi_test
		test/multi_test.cc
		)

	target_link_libraries(multi_test
		hesai_sdk_lib
		)
endif()
