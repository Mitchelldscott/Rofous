cmake_minimum_required(VERSION 3.10)

# set the project name
project(Rofous VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(Rofous test/AD_U_Test.cpp src/main.cpp)

target_include_directories(Rofous 
	PRIVATE
	${PROJECT_SOURCE_DIR}/include
)