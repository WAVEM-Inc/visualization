# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/node_editor_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/node_editor_autogen.dir/ParseCache.txt"
  "lib/EventBus/lib/CMakeFiles/EventBus_autogen.dir/AutogenUsed.txt"
  "lib/EventBus/lib/CMakeFiles/EventBus_autogen.dir/ParseCache.txt"
  "lib/EventBus/lib/EventBus_autogen"
  "node_editor_autogen"
  )
endif()
