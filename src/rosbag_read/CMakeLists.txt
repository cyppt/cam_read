cmake_minimum_required(VERSION 2.8.3)
project(rosbag_read)

## auto find yaml path
set(YAML_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../param.yaml)
set(BAG_FORMAT_YAML ${CMAKE_CURRENT_SOURCE_DIR}/../../bag_format.yaml)
configure_file(include/common/find_yaml_paht.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/common/find_yaml_paht.h)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        rosbag
        std_msgs
        geometry_msgs
        message_generation
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)


add_message_files(
        DIRECTORY sensor/msg
        FILES
        INSMsg.msg
        INSStdMsg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)


## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include/common
        ${PROJECT_SOURCE_DIR}/include/rosbag_read2
)


add_library(common_fun #此处为库的名称，一般即为头文件名称（不带后缀）
        src/common.cpp
        src/main.cpp
        src/rosbag_format.cpp
        include/rosbag_format/rosbag_format.h #指明库对应的源文件路径
)

add_library(rosbag_reader_class
        src/rosbag_reader2.cpp
)

add_library(rosbag_format_class
        src/rosbag_format.cpp
)

add_dependencies(common_fun
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(rosbag_reader_class
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(rosbag_format_class
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(common_fun
        ${catkin_LIBRARIES}
)

target_link_libraries(rosbag_reader_class
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Eigen_LIBRARIES}
        jsoncpp
        common_fun
        yaml-cpp
)

target_link_libraries(rosbag_format_class
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Eigen_LIBRARIES}
        jsoncpp
        common_fun
        yaml-cpp
)

## Declare a C++ executable
add_executable(rosbag_reader src/rosbag_reader.cpp
        )

add_executable(rosbag_reader2 src/main.cpp
        )

## Specify libraries to link a library or executable target against
target_link_libraries(rosbag_reader
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}  ## Add this line if using OpenCV
)

target_link_libraries(rosbag_reader2
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}  ## Add this line if using OpenCV
        ${Eigen_LIBRARIES}
        rosbag_reader_class
        rosbag_format_class
        )