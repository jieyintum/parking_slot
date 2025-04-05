cmake_minimum_required(VERSION 3.4.1)

include_directories(
         ${Boost_INCLUDE_DIR}
         ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ./
        ../common
        ../common_lib/include
)

add_executable(fusion_pse
                ../main.cpp
                ../common/utils/geom_utils.cpp
                pse_node.cpp
                pld_fusion/pld_fusion.cpp
                pld_fusion/slots_fusion.cpp
                pld_fusion/pld_slot_match.cpp
                pld_fusion/slots_openLine_calc.cpp
                arrow_fusion/arrow_fusion.cpp
                zebra_fusion/zebra_fusion.cpp
                speed_bump_fusion/speed_bump_fusion.cpp
                lane_fusion/lane_fusion.cpp
                lane_fusion/Astar/astar.cpp
                lane_fusion/poly_line/poly_line_fitting.cpp
)

if (USING_GPERFTOOLS_HEAP)
    target_link_libraries(fusion_pse
        tcmalloc
    )
endif()
#set(OPENCV_LIBS /root/cem_build/external/common_lib/OpenCV/arm/OpenCV-4.2.0/lib/libopencv_core.so
#        /root/cem_build/external/common_lib/OpenCV/arm/OpenCV-4.2.0/lib/libopencv_imgcodecs.so
#        /root/cem_build/external/common_lib/OpenCV/arm/OpenCV-4.2.0/lib/libopencv_imgproc.so)

set(OPENCV_LIBS -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc)

target_link_libraries(fusion_pse ${Boost_LIBRARIES}${OPENCV_LIBS}
)

ament_target_dependencies(fusion_pse
        rclcpp std_msgs cem_interfaces loc_msgs
        nav_msgs vision_msgs visualization_msgs
        mpc_prk_msgs cem_park_msgs diagnostic_msgs
        uss_msgs parameter_info_msgs vehicle_msgs
        Boost OpenCV)
