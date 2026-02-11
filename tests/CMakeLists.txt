include(FetchContent)
#Get catch2
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.12.0 # version
)
# Downloads and adds the subdirectory automatically
FetchContent_MakeAvailable(catch2)
add_executable(tests test_quaternion.cpp)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain quaternion_class)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})