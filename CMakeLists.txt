cmake_minimum_required(VERSION 3.8)
project( LineDetect )

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -march=native")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(BUILD_SHARED_LIBS ON)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


## OpenCV
find_package( OpenCV 4.2.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

include_directories( ${PROJECT_SOURCE_DIR}/include )

## Shared Lib
add_library(EDLine src/EDLine.cpp src/Gradient.cpp include/Utility.hpp)
target_link_libraries(EDLine ${OpenCV_LIBS} )


add_executable(time_cost app/time_cost.cpp )
target_link_libraries(time_cost EDLine)

add_executable(show app/show.cpp)
target_link_libraries(show EDLine)
