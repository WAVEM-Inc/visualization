# Install script for directory: /home/antique/Desktop/gui/node_editor/lib/EventBus/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/libEventBus.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES
    "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/generated/EventBusConfig.cmake"
    "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/generated/EventBusConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus/EventBusTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus/EventBusTargets.cmake"
         "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/CMakeFiles/Export/65caa07b9f76f7a4c8a94cfdcc0d26ef/EventBusTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus/EventBusTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus/EventBusTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/CMakeFiles/Export/65caa07b9f76f7a4c8a94cfdcc0d26ef/EventBusTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/EventBus" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/cmake-build-debug/lib/EventBus/lib/CMakeFiles/Export/65caa07b9f76f7a4c8a94cfdcc0d26ef/EventBusTargets-debug.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/EventBus.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/Bus.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/internal" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/internal/event_id.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/internal" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/internal/listener_traits.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/internal" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/internal/ListenerAttorney.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/Listener.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/perk" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/perk/PassPerk.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/perk" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/perk/Perk.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/perk" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/perk/PerkEventBus.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/perk" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/perk/TagPerk.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/perk" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/perk/WaitPerk.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/permission" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/permission/PostponeBus.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/stream" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/stream/EventStream.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dexode/eventbus/stream" TYPE FILE PERMISSIONS OWNER_EXECUTE OWNER_READ FILES "/home/antique/Desktop/gui/node_editor/lib/EventBus/lib/src/dexode/eventbus/stream/ProtectedEventStream.hpp")
endif()

