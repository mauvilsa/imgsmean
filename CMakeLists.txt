cmake_minimum_required( VERSION 2.8.4 )
project( imgsmean )
set( tool_EXE imgsmean )
include( FindPkgConfig )
pkg_check_modules( MagickWand REQUIRED MagickWand>=6.8 )

file( GLOB tool_SRC "*.c" )
add_executable( ${tool_EXE} ${tool_SRC} )

string( REPLACE ";" " " CFLAGS_STR "-Wall -W ${MagickWand_CFLAGS}" )
set_target_properties( ${tool_EXE} PROPERTIES COMPILE_FLAGS "${CFLAGS_STR}" )

include_directories( SYSTEM ${MagickWand_INCLUDEDIR} ) # To suppress system header warnings

target_link_libraries( ${tool_EXE} ${MagickWand_LDFLAGS} m )

install( TARGETS ${tool_EXE} DESTINATION bin )

add_custom_target( realclean cd ${CMAKE_HOME_DIRECTORY} COMMAND rm -fr ${tool_EXE} ${tool_EXE}.exe CMakeFiles CMakeCache.txt cmake_install.cmake install_manifest.txt Makefile )
