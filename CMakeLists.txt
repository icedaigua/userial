CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

set( CMAKE_CXX_COMPILER "g++")
set( CMAKE_C_COMPILER "gcc")

PROJECT( userial )
set( CMAKE_BUILD_TYPE Release  )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lm -Wall -O3 -march=native" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -lm -O3 -march=native" )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/inc )

# find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
src/protocol.c
src/serialib.c
src/gcs_thread.c
src/iofunction.c
)


target_link_libraries(${PROJECT_NAME}
pthread)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(main
src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})


add_executable(iofunc
src/iofunction.c)
target_link_libraries(iofunc)
