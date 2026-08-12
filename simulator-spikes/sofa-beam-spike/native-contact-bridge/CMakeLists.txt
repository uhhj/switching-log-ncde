cmake_minimum_required(VERSION 3.22)

project(NativeContactBridge VERSION 0.1.0 LANGUAGES CXX)

find_package(Sofa.Framework REQUIRED)
find_package(Sofa.Simulation.Core REQUIRED)

set(NCB_SRC src/NativeContactBridge)

add_library(NativeContactBridge SHARED
    ${NCB_SRC}/api.h
    ${NCB_SRC}/init.h
    ${NCB_SRC}/NativeContactBridge.h
    ${NCB_SRC}/init.cpp
    ${NCB_SRC}/NativeContactBridge.cpp
)

target_compile_features(NativeContactBridge PRIVATE cxx_std_17)
target_compile_definitions(NativeContactBridge PRIVATE NATIVE_CONTACT_BRIDGE_BUILD)
target_include_directories(NativeContactBridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(NativeContactBridge PRIVATE Sofa.Core Sofa.Simulation.Core)
set_target_properties(NativeContactBridge PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
