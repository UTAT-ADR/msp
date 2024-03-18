cmake_minimum_required(VERSION 3.5)
project(msp)
set(CMAKE_CXX_STANDARD 20)

set(PROJECT_DESCRIPTION "Implementation of the MultiWii Serial Protocol (MSP) for MultiWii and Cleanflight flight controller")

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 3.3.4 QUIET)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# elseif(MSVC)
#   add_compile_options(/W4 /WX)
# endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} \
    -fsanitize=address \
    -fsanitize=bool \
    -fsanitize=bounds \
    -fsanitize=enum \
    -fsanitize=float-cast-overflow \
    -fsanitize=float-divide-by-zero \
    -fsanitize=nonnull-attribute \
    -fsanitize=returns-nonnull-attribute \
    -fsanitize=signed-integer-overflow \
    -fsanitize=undefined \
    -fsanitize=vla-bound \
    -fno-sanitize=alignment \
    -fsanitize=leak \
    -fsanitize=object-size \
")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ASIO REQUIRED)

if(UNIX)
    set(BUILD_SHARED_LIBS "ON")
endif()

add_definitions(-DASIO_STANDALONE)
add_definitions(-DASIO_HAS_STD_ADDRESSOF)
add_definitions(-DASIO_HAS_STD_ARRAY)
add_definitions(-DASIO_HAS_CSTDINT)
add_definitions(-DASIO_HAS_STD_SHARED_PTR)
add_definitions(-DASIO_HAS_STD_TYPE_TRAITS)

OPTION(BUILD_EXAMPLES "Build Library with examples" OFF)
OPTION(BUILD_TESTS "Build Library with tests" OFF)

find_package(Threads)

# set(MSP_SOURCE_DIR src)
# set(MSP_INCLUDE_DIR include/msp)
# include_directories(${MSP_INCLUDE_DIR})

catkin_package(INCLUDE_DIRS include)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

################################################################################
### libraries

# client library
cs_add_library(mspclient src/Client.cpp src/PeriodicTimer.cpp)
target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT} ASIO::ASIO)

# high-level API
cs_add_library(msp_fcu src/FlightController.cpp)
target_link_libraries(msp_fcu mspclient)


### executables

cs_add_executable(betaflight_node src/betaflight_node.cpp)
cs_add_executable(motor_test src/motor_test.cpp)

target_link_libraries(betaflight_node
    ${catkin_LIBRARIES} -lstdc++fs
    mspclient
)

target_link_libraries(motor_test
    ${catkin_LIBRARIES} -lstdc++fs
    mspclient
)

################################################################################
### examples / tests

if(BUILD_EXAMPLES)

    # testing publish/subscribe
    cs_add_executable(fcu_test examples/fcu_test.cpp)
    target_link_libraries(fcu_test msp_fcu)

    # test setting motors directly
    cs_add_executable(fcu_motors examples/fcu_motor_test.cpp)
    target_link_libraries(fcu_motors msp_fcu)

    # subscribing with custom type
    cs_add_executable(fcu_custom_type examples/fcu_custom_type.cpp)
    target_link_libraries(fcu_custom_type msp_fcu)

    # client test for asynchronous callbacks
    cs_add_executable(client_async_test examples/client_async_test.cpp)
    target_link_libraries(client_async_test mspclient)

    # client test for blocking read
    cs_add_executable(client_read_test examples/client_read_test.cpp)
    target_link_libraries(client_read_test mspclient)

endif()

################################################################################
### installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(DIRECTORY ${MSP_INCLUDE_DIR} DESTINATION include/ FILES_MATCHING PATTERN "*.hpp")

SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib")
SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include/")
SET(PKG_CONFIG_LIBS         "-L\${libdir} -lmsp_fcu -lmspclient")
SET(PKG_CONFIG_CFLAGS       "-I\${includedir}")

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
)

INSTALL(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
        DESTINATION lib/pkgconfig)



###############################################################################
### testing
if(BUILD_TESTS)
    enable_testing()

    # Download and unpack googletest at configure time
    configure_file(CMakeLists.txt.in googletest/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest)
    if(result)
      message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest)
    if(result)
      message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(/usr/src/gtest
                     ${CMAKE_BINARY_DIR}/googletest-build
                     EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include")
    endif()

    cs_add_executable(value_test test/value_test.cpp)
    target_link_libraries(value_test gtest_main)
    add_test(NAME value_test COMMAND value_test)

    cs_add_executable(bytevector_test test/ByteVector_test.cpp)
    target_link_libraries(bytevector_test gtest_main)
    add_test(NAME bytevector_test COMMAND bytevector_test)

endif()

# Finish
cs_install()
cs_export()
