cmake_minimum_required(VERSION 3.4...3.18)
project(intelligentCar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


set("${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/src")
include_directories(${COMMON_INCLUDE_DIR})

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})


# CameraDisplay （相机查验）
set(CAMERA_DISPLAY_PROJECT_NAME "camera_display")
set(CAMERA_DISPLAY_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_display.cpp)
add_executable(${CAMERA_DISPLAY_PROJECT_NAME} ${CAMERA_DISPLAY_PROJECT_SOURCES})
# target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# CameraDisplay （相机采片）
set(CAMERA_DISPLAY_PROJECT_NAME "camera_collection")
set(CAMERA_DISPLAY_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_collection.cpp)
add_executable(${CAMERA_DISPLAY_PROJECT_NAME} ${CAMERA_DISPLAY_PROJECT_SOURCES})
# target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# ImageCalibration（相机标定）
set(CALIBRATION_PROJECT_NAME "camera_calibration")
set(CALIBRATION_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_calibration.cpp)
add_executable(${CALIBRATION_PROJECT_NAME} ${CALIBRATION_PROJECT_SOURCES})
target_link_libraries(${CALIBRATION_PROJECT_NAME}  ${OpenCV_LIBS})

# PaddleDetection (元素识别)
set(PROJECT_NAME "paddle_detection")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/tools/paddle_detection.cpp)
target_link_libraries(${ICAR_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME}  pthread )


# ICAR (智能车主程序)
set(PROJECT_NAME "icar")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/icar.cpp)
target_link_libraries(${ICAR_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME}  pthread )
target_link_libraries(${ICAR_PROJECT_NAME}  ${SERIAL_LIBRARIES})
