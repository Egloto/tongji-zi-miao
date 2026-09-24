cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/home/egloto/.local/openvino_toolkit/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)