include(${CMAKE_CURRENT_LIST_DIR}/mymath-targets.cmake)
get_filename_component(MYMATH_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../include/mymath" ABSOLUTE)
include_directories(${MYMATH_INCLUDE_DIRS})
set(MYMATH_LIBRARIES mymath)
