cmake_minimum_required(VERSION 3.20)
project(
	quaternion_test
	VERSION 1.0
	LANGUAGES CXX
)
#C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
add_subdirectory(src)
add_subdirectory(tests)
