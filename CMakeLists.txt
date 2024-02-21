cmake_minimum_required(VERSION 3.27)
project(node_editor)

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

add_executable(node_editor
        main.cpp
        includes/utils/coordinate_handler.h
        includes/model/Position.h
        includes/model/SensingRange.h
        includes/model/Link.h
        includes/model/Node.h
        includes/model/MapNode.h
        includes/model/Path.h
        src/ui/map/map_view.cpp
        includes/ui/map/map_view.h
        src/ui/main/main_window.cpp
        includes/ui/main/main_window.h
        src/ui/menu/menu_bar.cpp
        includes/ui/menu/menu_bar.h
        includes/patterns/observer/singleton.h
        includes/patterns/observer/observer.h
        includes/patterns/observer/subject.h
        includes/patterns/observer/ConcreteObserver.h
        src/viewmodel/map_node_view_model.cpp
        includes/viewmodel/map_node_view_model.h
        src/model/MapNode.cpp
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

include_directories(node_editor
        includes
        includes/ui/map
)

target_link_libraries(node_editor
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::WebEngineWidgets
        Qt5::WebChannel
        nlohmann_json::nlohmann_json
)

