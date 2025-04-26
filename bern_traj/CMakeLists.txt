cmake_minimum_required(VERSION 3.8)
project(bern_traj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(TBB REQUIRED)

if(TBB_FOUND)
  message("Found TBB!")
endif()

find_package(osqp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(dependencies
  rclcpp
  rclpy
  Eigen3
  nav_msgs
  uav_msgs
  osqp
  TBB
)

##OSQP INCLUDE DIRECTORIES
find_path(OSQP_INCLUDE_DIR NAMES
	osqp/auxil.h
	osqp/constants.h
	osqp/cs.h
	osqp/ctrlc.h
	osqp/error.h
	osqp/glob_opts.h
	osqp/lin_alg.h
	osqp/lin_sys.h
	osqp/osqp_configure.h
	osqp/osqp.h
	osqp/polish.h
	osqp/proj.h
	osqp/scaling.h
	osqp/types.h
	osqp/util.h
	osqp/version.h
    HINTS /usr/local/include
)

set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR} )

list(APPEND TRAJECTORIES
  src/bernstein_trajectory.cpp
  )

add_library(trjectoryLibNumOpt SHARED ${TRAJECTORIES})
ament_target_dependencies(trjectoryLibNumOpt ${dependencies})
target_link_libraries(trjectoryLibNumOpt ${OSQP_INCLUDE_DIR} osqp::osqp osqp::osqpstatic)

add_executable(trajectory_manager_num_opt src/trajectory_manager.cpp)
ament_target_dependencies(trajectory_manager_num_opt ${dependencies})
target_link_libraries(trajectory_manager_num_opt trjectoryLibNumOpt osqp::osqp)

add_executable(dqp src/optimizer.cpp)
ament_target_dependencies(dqp ${dependencies})
target_link_libraries(dqp trjectoryLibNumOpt Eigen3::Eigen TBB::tbb)

install(TARGETS
          trjectoryLibNumOpt
          trajectory_manager_num_opt
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
          include/
        DESTINATION include)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
