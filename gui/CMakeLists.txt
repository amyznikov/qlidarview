CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(qlidarview-gui CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if( NOT CMAKE_CXX_STANDARD ) 
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


# ----------------------------------------------------------------------------
# target sources

file(GLOB_RECURSE SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
 
file(GLOB_RECURSE HEADER_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

file(GLOB_RECURSE QRC_FILES
	"${CMAKE_CURRENT_SOURCE_DIR}/*.qrc" )


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS} 
    ${CORE_INCLUDES})


# AUTOMOC problem is that it merges all mocs into single file
# Single automoc file prevents later dependency-only linking.
# It is why I schedule the moc-ing manually here.
set(CMAKE_AUTOMOC OFF)
foreach(_header ${HEADER_FILES})

	file(STRINGS "${_header}" _strings REGEX "Q_OBJECT")

	if( NOT "${_strings}" STREQUAL "" )
    	file(RELATIVE_PATH _relpath ${CMAKE_CURRENT_SOURCE_DIR} "${_header}")
		set(_mocfile "${CMAKE_CURRENT_BINARY_DIR}/${_relpath}.moc.cc")
		qt5_generate_moc("${_header}" "${_mocfile}")
    	list(APPEND SRC_FILES "${_mocfile}")

		# include(AddFileDependencies)
    	# add_file_dependencies("${_impl}" "${_mocfile}")
    endif()

endforeach()


foreach(QRC_FILE ${QRC_FILES})
	qt5_add_resources(RESOURCE_FILES 
		${QRC_FILE})
endforeach()

message("RESOURCE_FILES =${RESOURCE_FILES}")

add_library(${PROJECT_NAME} STATIC 
	${HEADER_FILES} 
	${SRC_FILES} 
	${RESOURCE_FILES})


install(TARGETS ${PROJECT_NAME} DESTINATION lib)
foreach ( file ${HEADER_FILES} )
   	get_filename_component( dir ${file} DIRECTORY )
    file(RELATIVE_PATH relpath "${CMAKE_CURRENT_SOURCE_DIR}" ${dir})
    install( FILES ${file} DESTINATION include/qlidarview/gui/${relpath} )
endforeach()

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
