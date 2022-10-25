cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

include_directories(utils)

add_library(custom STATIC "src/utils.c")

set_target_properties(custom PROPERTIES PUBLIC_HEADER
"utils/*.h")
