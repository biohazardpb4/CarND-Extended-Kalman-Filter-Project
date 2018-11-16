include_directories("${PROJECT_SOURCE_DIR}")

set(SOURCE_FILES ../src/tools.cpp ../src/FusionEKF.cpp ../src/kalman_filter.cpp ../src/tools.h ../src/FusionEKF.h ../src/kalman_filter.h)

add_executable(tools tools.cpp ${SOURCE_FILES})
target_link_libraries(tools gtest)
add_test(NAME tools_test COMMAND tools)
