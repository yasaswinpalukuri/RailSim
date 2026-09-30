include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(railsim_tests
    unit/track_graph_test.cpp
    unit/config_loader_test.cpp
    unit/route_planner_test.cpp
    unit/event_logger_test.cpp
    unit/interlocking_test.cpp
    unit/pid_controller_test.cpp
    unit/train_test.cpp
)
target_link_libraries(railsim_tests PRIVATE railsim_core railsim_warnings GTest::gtest_main)

# Lets tests open the example files without depending on the working directory.
target_compile_definitions(railsim_tests PRIVATE
    RAILSIM_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")

include(GoogleTest)
gtest_discover_tests(railsim_tests)
