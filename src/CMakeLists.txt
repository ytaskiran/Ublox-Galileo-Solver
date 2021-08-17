cmake_minimum_required(VERSION 3.16)
project(GalileoParser VERSION 0.1)

add_library(galileo_parser
            galileo_parser.h
            galileo_parser.cc)   
   

add_executable(galileo main.cc)

target_link_libraries(galileo 
                      PRIVATE 
                      galileo_parser)
