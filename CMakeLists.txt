cmake_minimum_required(VERSION 3.22.1)

project(liegroups VERSION 0.0.1 LANGUAGES CXX)

include(GNUInstallDirs)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


################
# DEPENDENCIES
################
find_package(Eigen3 3.4 REQUIRED)


############
# INCLUDES
############
include_directories(include)


############
# LIBRARIES
############
add_library(liegroups INTERFACE)
target_include_directories(liegroups INTERFACE
	$<BUILD_INTERFACE:${liegroups_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

########
# LINK
########
target_link_libraries(liegroups INTERFACE
	Eigen3::Eigen
)


###########
# Examples
###########

add_executable(example 
	examples/main.cpp
	)
target_link_libraries(example
	liegroups
	)


#########
# GTest
#########

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable( liegroups_test 
	tests/liegroups_test.cpp
)

target_link_libraries(liegroups_test
  liegroups
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(liegroups_test)


