cmake_minimum_required(VERSION 3.12)

project(liegroups VERSION 0.0.1)

include(GNUInstallDirs)


# # GoogleTest requires at least C++14
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(BUILD_SHARED_LIBS ON)


#####################
## DEPENDENCIES
#####################
find_package(Eigen3 3.4 REQUIRED)


######################
## CREATE LIBRARY
######################

add_library(liegroups INTERFACE)

target_include_directories(liegroups INTERFACE 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(liegroups INTERFACE
	Eigen3::Eigen
	)


####################
## INSTALL LIBRARY
####################

install(TARGETS liegroups
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file("liegroupsConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(EXPORT ${PROJECT_NAME}_Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)


install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/liegroups DESTINATION include)



####################
## TEST EXECUTABLE
####################

add_executable(example
	examples/main.cpp)
target_link_libraries(example
	liegroups
	Eigen3::Eigen
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
