cmake_minimum_required(VERSION 3.11)
project(sophia)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BOOST_ROOT /usr/local)
find_package(Boost 1.66 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

include(GoogleTest)

include_directories(inc)

add_definitions(-DSOPHIA_PWHASH_FAST -DSOPHIA_EXTRA_API)

file(GLOB_RECURSE SOPHIA_INC "inc/**.hpp")
file(GLOB_RECURSE SOPHIA_SRC "src/**.cpp")
file(GLOB_RECURSE SOPHIA_GTESTS_INC "tst/gtests/**.hpp")
file(GLOB_RECURSE SOPHIA_GTESTS_SRC "tst/gtests/**.cpp")
#file(GLOB_RECURSE SOPHIA_GTESTS_SRC "tst/gtests/pubsub.cpp" "tst/gtests/Network.cpp")

add_library(sophia ${SOPHIA_INC} ${SOPHIA_SRC})
target_link_libraries(sophia
    pthread stdc++fs
    ${Boost_LIBRARIES}
    sodium sqlite3)

add_executable(sophia-cli cli/main.cpp)
target_link_libraries(sophia-cli sophia miniupnpc)

add_executable(sophia-tst-net tst/network.cpp)
target_link_libraries(sophia-tst-net sophia)

add_executable(sophia-gtests ${SOPHIA_GTESTS_INC} ${SOPHIA_GTESTS_SRC})
target_link_libraries(sophia-gtests sophia gtest gtest_main)
gtest_add_tests(TARGET sophia-gtests AUTO)
