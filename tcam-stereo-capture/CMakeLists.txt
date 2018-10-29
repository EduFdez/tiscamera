project(tcam-stereo C CXX)
 
cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

set(TISCAMERA_DIR "${PROJECT_SOURCE_DIR}/..")
if(NOT IS_DIRECTORY "${TISCAMERA_DIR}/examples/cpp/common" )
    message(FATAL_ERROR "Error : TISCAMERA_DIR not set to tiscamera directory. Please correct CMakeLists.txt." )
    return()
endif()

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB tcam)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${TISCAMERA_DIR}/examples/cpp/common  ${GSTREAMER_INCLUDE_DIRS} ${TCAM_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
add_definitions(${GSTREAMER_CFLAGS_OTHER})  


##################################################################################################################
###						Stereo software-trigget recorder
##################################################################################################################
add_executable(stereocapture_onKeyPress stereocapture_onKeyPress.cpp ${TISCAMERA_DIR}/examples/cpp/common/tcamcamera.cpp )
target_link_libraries(stereocapture_onKeyPress ${TCAMLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} ${OpenCV_LIBS} )


##################################################################################################################
###						Stereo capture with IMU external-trigger (Xsens)
##################################################################################################################
include_directories("/usr/local/xsens/examples/mtsdk/src_cpp/include" )
link_directories("/usr/local/xsens/examples/mtsdk/src_cpp/xcommunication" "/usr/local/xsens/examples/mtsdk/src_cpp/xstypes")

add_executable(imu_stereocapture    imu_xsens_stereocapture.cpp
                                    xsens/deviceclass.cpp 
									xsens/deviceclass.h 
									xsens/conio.c 
									xsens/conio.h
                                    ${TISCAMERA_DIR}/examples/cpp/common/tcamcamera.cpp
				)
target_link_libraries(imu_stereocapture "-lxcommunication -lxstypes -lpthread -lrt -ldl" ) 
target_link_libraries(imu_stereocapture ${TCAMLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} ${OpenCV_LIBS} -pthread )


##################################################################################################################
###						Stereo software-trigger recorder
##################################################################################################################
add_executable(stereo_software_trigger stereo_software_trigger.cpp ${TISCAMERA_DIR}/examples/cpp/common/tcamcamera.cpp )
target_link_libraries(stereo_software_trigger ${TCAMLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} ${OpenCV_LIBS} -pthread )
install(TARGETS stereo_software_trigger RUNTIME DESTINATION bin)



