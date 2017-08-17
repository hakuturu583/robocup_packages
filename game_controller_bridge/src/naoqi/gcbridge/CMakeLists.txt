cmake_minimum_required(VERSION 2.8)
project(gcbridge)

find_package(qibuild)

# Create a executable named gcbridge
# with the source file: main.cpp
qi_create_bin(gcbridge "main.cpp")

#qi_create_lib(gcbridge SHARED "main_lib.cpp" )

qi_use_lib(gcbridge ALCOMMON ALPROXIES BOOST BOOST_SYSTEM BOOST_THREAD QI ROS)

set(CMAKE_CXX_FLAGS "-Wall")

# Add a simple test:
#qi_create_test(test_gcbridge "test.cpp")
