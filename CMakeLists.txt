cmake_minimum_required(VERSION 3.8)
project(estimation_assignment)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/occupancy_grid_mapper.py
  scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME})

# Imported by occupancy_grid_mapper.py, so it must sit next to it. (With
# --symlink-install the installed script is a symlink and Python looks next
# to its target, scripts/; this copy covers plain installs.)
install(FILES scripts/transformations.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# localization.py is a plain Python program,
# run in place with python3; laser_and_odometry/ is played from the source
# tree with `ros2 bag play`. Neither is installed.

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_transformations test/test_transformations.py)
  ament_add_pytest_test(test_occupancy_grid_map test/test_occupancy_grid_map.py)
  ament_add_pytest_test(test_bag test/test_bag.py)
endif()

ament_package()
