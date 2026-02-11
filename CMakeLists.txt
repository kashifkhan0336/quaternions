cmake_minimum_required(VERSION 3.20)
project(
	quaternion_test
	VERSION 1.0
	LANGUAGES CXX
)
#C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)
#Get catch2
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.12.0 # version
)
# Downloads and adds the subdirectory automatically
FetchContent_MakeAvailable(catch2)
add_executable(quaternion_test main.cpp)
target_link_libraries(quaternion_test PRIVATE Catch2::Catch2WithMain)
#adding extra directory of catch2 to path
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
CATCH_DISCOVER_TESTS(quaternion_test)