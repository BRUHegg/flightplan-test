cmake_minimum_required(VERSION 3.13)
project(libnav_test CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -fPIC -g")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory("fpln")

file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FILE(GLOB HDR_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
add_executable(libnavtest ${SRC_FILES} ${HDR_FILES})

set_target_properties(libnavtest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}")

if(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0)
    add_definitions(-D_WIN32_WINNT=0x0600)
    set(ARCH "mingw64")
    
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")
    set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static-libgcc -s")
    set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS}  -static-libgcc -static-libstdc++ -s")
else()
    set(ARCH "lin64")
endif()
include(CMakePrintHelpers)
option(LIBNAV "/home/betatest/Documents/programming/libnav-redist" "")
cmake_print_variables(LIBNAV)
target_include_directories(libnavtest PUBLIC "${LIBNAV}/include")
target_include_directories(fpln PUBLIC "${LIBNAV}/include")
file(GLOB LIBNAV_LIBS ${LIBNAV}/${ARCH}/*.a)
cmake_print_variables(LIBNAV_LIBS)
target_link_libraries(fpln PUBLIC ${LIBNAV_LIBS})
target_link_libraries(libnavtest PUBLIC fpln)
