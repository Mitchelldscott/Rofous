cmake_minimum_required(VERSION 3.10)

# set the project name
project(Rofous VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(Rofous
	src/Aerial_Device.cpp 
	src/U_Test.cpp 
	src/main.cpp
)

set_target_properties(Rofous
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

target_include_directories(Rofous 
	PRIVATE
	${PROJECT_SOURCE_DIR}/include
)