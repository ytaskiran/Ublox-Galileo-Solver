cmake_minimum_required(VERSION 3.16)
project(GalileoSolver VERSION 0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(galileo_solver
            galileo_solver.h
            galileo_solver.cc)   
   

add_executable(galileo main.cc)

target_link_libraries(galileo 
                      PRIVATE 
                      galileo_solver)
