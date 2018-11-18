include_directories("${PROJECT_SOURCE_DIR}")

set(SOURCE_FILES ../src/tools.cpp ../src/FusionEKF.cpp ../src/kalman_filter.cpp ../src/tools.h ../src/FusionEKF.h ../src/kalman_filter.h)

add_executable(kalman_filter kalman_filter.cpp ${SOURCE_FILES})
target_link_libraries(kalman_filter gtest)
add_test(NAME kalman_filter_test COMMAND kalman_filter)

add_executable(tools tools.cpp ${SOURCE_FILES})
target_link_libraries(tools gtest)
add_test(NAME tools_test COMMAND tools)

add_executable(fusion_ekf FusionEKF.cpp ${SOURCE_FILES})
target_link_libraries(fusion_ekf gtest)
add_test(NAME fusion_ekf_test COMMAND fusion_ekf)
