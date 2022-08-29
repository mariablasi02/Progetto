cmake_minimum_required(VERSION 3.16)
project(boids VERSION 0.1.0)

# enabled test support
include(CTest)

# requested the use of the Standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# enabled warnings
string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

# enabled sanitizer 
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")

# requested SFML graphics 
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

# added executable boids-sfml
add_executable(boids-sfml main.cpp boids.cpp operators.cpp rulesofswim.cpp)
# dependencies from sfml.graphics
target_link_libraries(boids-sfml PRIVATE sfml-graphics)

#  if testing is enabled
if (BUILD_TESTING)

  # added executable boids.t
  add_executable(boids.t boids.test.cpp boids.cpp operators.cpp rulesofswim.cpp)
  # added executable boids.t to test list
  add_test(NAME boids.t COMMAND boids.t)

endif()