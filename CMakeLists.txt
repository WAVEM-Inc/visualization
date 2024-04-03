cmake_minimum_required(VERSION 3.22.1)
project(route_editor)

if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
set(RESOURCE_DIR_DEST "${CMAKE_BINARY_DIR}/resources")

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        WebEngineWidgets
        WebChannel
        REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(third_party/geographiclib)

add_executable(route_editor
        main.cpp
        includes/utils/coordinate_handler.h
        includes/struct/Position.h
        includes/struct/DetectionRange.h
        includes/struct/Link.h
        includes/struct/Node.h
        includes/struct/GraphNode.h
        includes/struct/Path.h
        src/ui/map/map_view.cpp
        includes/ui/map/map_view.h
        src/ui/main/main_window.cpp
        includes/ui/main/main_window.h
        src/ui/menu/menu_bar.cpp
        includes/ui/menu/menu_bar.h
        includes/utils/patterns/singleton/singleton.h
        includes/utils/patterns/observer/observer.h
        includes/utils/patterns/observer/subject.h
        src/struct/GraphNode.cpp
        includes/struct/RouteFile.h
        src/struct/Position.cpp
        src/struct/Position.cpp
        src/struct/RouteFile.cpp
        src/struct/Link.cpp
        src/struct/Node.cpp
        src/struct/Path.cpp
        src/struct/DetectionRange.cpp
        includes/utils/events/FileEvents.h
        src/utils/file/route_file_reader.cpp
        includes/utils/file/route_file_reader.h
        src/utils/file/route_file_writer.cpp
        includes/utils/file/route_file_writer.h
        src/ui/editor/route_editor.cpp
        includes/ui/editor/route_editor.h
        includes/struct/NodeCode.h
        src/struct/NodeCode.cpp
        src/utils/file/code_file_reader.cpp
        includes/utils/file/code_file_reader.h
        src/ui/editor/node_list_model.cpp
        includes/ui/editor/node_list_model.h
        src/ui/editor/node_editor.cpp
        includes/ui/editor/node_editor.h
        includes/struct/FileInfo.h
        src/model/file_info_model.cpp
        includes/model/file_info_model.h
        src/model/path_info_model.cpp
        includes/model/path_info_model.h
        src/model/node_info_model.cpp
        includes/model/node_info_model.h
        src/model/map_node_model.cpp
        includes/model/map_node_model.h
        src/ui/dialog/node_type_dialog.cpp
        includes/ui/dialog/node_type_dialog.h
        src/ui/editor/detection_range_view.cpp
        includes/ui/editor/detection_range_view.h
        src/ui/editor/detection_range_list_view.cpp
        includes/ui/editor/detection_range_list_view.h
        src/model/map_state_model.cpp
        includes/model/map_state_model.h
        src/utils/file/config_file_reader.cpp
        includes/utils/file/config_file_reader.h
        includes/struct/ConfigFile.h
        src/struct/ConfigFile.cpp
        src/utils/file/config_file_writer.cpp
        includes/utils/file/config_file_writer.h
        src/ui/editor/node_list_view.cpp
        includes/ui/editor/node_list_view.h
        includes/struct/Code.h
        src/struct/Code.cpp
        includes/struct/CodeGroup.h
        src/struct/CodeGroup.cpp
        includes/enum/CodeType.h
        src/enum/CodeType.cpp
        src/model/code_info_model.cpp
        includes/model/code_info_model.h
        src/ui/menu/node_popup_menu.cpp
        includes/ui/menu/node_popup_menu.h
        src/ros2/nav_sat_fix_subscriber.cpp
        includes/ros2/nav_sat_fix_subscriber.h
        src/model/ros_2_data_model.cpp
        includes/model/ros_2_data_model.h
        includes/utils/GeoPositionUtil.h
        src/utils/GeoPositionUtil.cpp
)

include(FetchContent)

FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.10.5
)

FetchContent_MakeAvailable(json)

file(COPY ${RESOURCE_DIR} DESTINATION ${CMAKE_BINARY_DIR})
install(DIRECTORY ${RESOURCE_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX})
add_definitions(-DRESOURCE_DIR="${RESOURCE_DIR_DEST}")

include_directories(route_editor
        includes
        includes/ui/map
)

target_link_libraries(route_editor
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::WebEngineWidgets
        Qt5::WebChannel
        nlohmann_json::nlohmann_json
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        GeographicLib
)

