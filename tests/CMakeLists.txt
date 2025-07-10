enable_testing()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(test_quaternion_constructors test.cpp)

target_include_directories(test_quaternion_constructors
  PRIVATE ${PROJECT_SOURCE_DIR}/include/quaternionlib
)

target_link_libraries(test_quaternion_constructors
  PRIVATE
  quaternionlib
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_quaternion_constructors)
