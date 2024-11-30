# CMakeList.txt : CMake project for tflow-capture, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()


project ("tflow-capture")

find_package(PkgConfig REQUIRED)

# Add source to this project's executable.
add_executable (tflow-capture
    "v4l2Device.cpp"
    "v4l2Device.h"
    "main.cpp"
    "tflow-common.h"
    "tflow-glib.hpp"
    "tflow-capture.cpp"
    "tflow-capture.h"
    "tflow-buf.h"
    "tflow-buf-srv.cpp"
    "tflow-buf-srv.h"
    "tflow-ctrl-srv.cpp"
    "tflow-ctrl-srv.h"
    "tflow-ctrl-cli-port.cpp"
    "tflow-ctrl-cli-port.h"
    "tflow-ctrl-capture.cpp"
    "tflow-ctrl-capture.h"
    "tflow-ctrl.h"
    "tflow-ctrl.cpp"
    "tflow-ap-fixar.cpp"
    "tflow-ap-fixar.h"
    "tflow-ap.cpp"
    "tflow-ap.h"
    "tflow-player.cpp"
    "tflow-player.h"
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET tflow-capture PROPERTY CXX_STANDARD 20)
endif()

pkg_check_modules(OTHERS REQUIRED IMPORTED_TARGET
    json11
    libjpeg
)

pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET 
    glib-2.0
    gmodule-2.0
    gio-2.0
    gio-unix-2.0
)

pkg_check_modules(GLIBMM REQUIRED IMPORTED_TARGET 
    glibmm-2.4
    giomm-2.4
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0") # Add debug information

target_include_directories(tflow-capture PRIVATE ${GLIB_INCLUDE_DIRS})
target_include_directories(tflow-capture PRIVATE ${GLIBMM_INCLUDE_DIRS})
target_include_directories(tflow-capture PRIVATE ${OTHERS_INCLUDE_DIRS})
 
target_link_libraries(tflow-capture PkgConfig::GLIB)
target_link_libraries(tflow-capture PkgConfig::GLIBMM)
target_link_libraries(tflow-capture PkgConfig::OTHERS)

target_link_libraries(tflow-capture -lrt)

# TODO: Add tests and install targets if needed.
