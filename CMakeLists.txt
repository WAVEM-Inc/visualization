cmake_minimum_required(VERSION 3.22.1)
project(route_editor)

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
        includes/enum/NodeType.h
        includes/enum/NodeKind.h
        includes/enum/Direction.h
        src/ui/dialog/node_type_dialog.cpp
        includes/ui/dialog/node_type_dialog.h
        src/enum/NodeType.cpp
        src/enum/NodeKind.cpp
        src/enum/Direction.cpp
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
)

