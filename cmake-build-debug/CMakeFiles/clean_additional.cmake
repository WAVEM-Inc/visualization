# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/route_editor_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/route_editor_autogen.dir/ParseCache.txt"
  "route_editor_autogen"
  )
endif()
