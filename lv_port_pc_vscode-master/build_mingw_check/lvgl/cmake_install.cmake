# Install script for directory: F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/lvgl")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/w64devkit/bin/objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/src" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\_private\\.h$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lv_conf.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lv_version.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lvgl.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/lib/liblvgl.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lv_conf.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lv_version.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lvgl.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkgconfig" TYPE FILE FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/lvgl.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/lib/liblvgl_thorvg.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lv_conf.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lv_version.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lvgl.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/demos" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/lib/liblvgl_demos.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lv_conf.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lv_version.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lvgl.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/examples" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/lib/liblvgl_examples.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lv_conf.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lv_version.h"
    "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/lvgl/lvgl.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.1/lv_port_pc_vscode-master/build_mingw_check/lvgl/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
