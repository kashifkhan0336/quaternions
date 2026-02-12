include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.17.0 # release-1.10.0
)
FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.9.5 # version
)
# Downloads and adds the subdirectory automatically
FetchContent_MakeAvailable(googletest benchmark)

add_library(quaternion_class quaternion.cpp)
target_include_directories(quaternion_class PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(main main.cpp)
target_link_libraries(main benchmark::benchmark)