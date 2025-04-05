cmake_minimum_required(VERSION 3.8)
project(parking_se_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DDEBUG_MODE=1) 

set(USING_GPERFTOOLS_HEAP OFF)

if (USING_GPERFTOOLS_HEAP)
    add_definitions(-DUSING_GPERFTOOLS_HEAP)
endif()


# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# add_compile_options(-O3 -g -Wall -fsanitize=address -fno-omit-frame-pointer)
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cem_interfaces REQUIRED)
find_package(loc_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mpc_prk_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(debug_msgs REQUIRED)
find_package(fdr_msgs REQUIRED)
find_package(cem_park_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(parameter_info_msgs REQUIRED)
find_package(uss_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(f100c_msgs REQUIRED)
find_package(Boost COMPONENTS regex system REQUIRED)
find_package(Eigen3 QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
#set(OpenCV_INCLUDE_DIRS "/root/cem_build/external/common_lib/OpenCV/arm/OpenCV-4.2.0/include/opencv4/")
set(OPENCV_INCLUDE_DIRS "/usr/include/opencv4/")

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

# ----------- add main target -----------


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(fusion_pse)


install(FILES README.md
        DESTINATION fusion_pse/
	)

install(TARGETS
        fusion_pse
        ARCHIVE DESTINATION fusion_pse/lib
        LIBRARY DESTINATION fusion_pse/lib
        RUNTIME DESTINATION fusion_pse/lib
        )

install(DIRECTORY script/
        DESTINATION fusion_pse/script
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY yaml/
  DESTINATION fusion_pse/config
  USE_SOURCE_PERMISSIONS
)

ament_package()
