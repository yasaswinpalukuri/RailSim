# All simulator logic lives in this library so that the CLI and the tests
# link against exactly the same code.
add_library(railsim_core
    track_graph.cpp
    config_loader.cpp
    route_planner.cpp
)
target_include_directories(railsim_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(railsim_core PRIVATE railsim_warnings)

add_executable(railsim main.cpp)
target_link_libraries(railsim PRIVATE railsim_core railsim_warnings)
