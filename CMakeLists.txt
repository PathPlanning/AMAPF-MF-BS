cmake_minimum_required(VERSION 3.8)
project(gr_offline_online)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
endif()

add_executable(flow_bs src/flow-bs.cpp)
