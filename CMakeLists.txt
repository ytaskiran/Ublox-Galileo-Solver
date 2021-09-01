cmake_minimum_required(VERSION 3.16)
project(GalileoSolver VERSION 0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)


add_library(galileo_solver src/galileo_solver.cpp)   
   
add_executable(galileo src/main.cc)

target_link_libraries(galileo 
                      PRIVATE 
                      galileo_solver)

enable_testing()

add_executable(galileo_test test/unit_tests.cpp)
target_link_libraries(galileo_test galileo_solver)
target_link_libraries(galileo_test gtest_main)

include(GoogleTest)
gtest_discover_tests(galileo_test)