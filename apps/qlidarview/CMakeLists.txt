CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
project(qlidarview CXX)

# check for mandatory dependencies 
if ( NOT QGLViewer_FOUND ) 
  message( "SKIP ${PROJECT_NAME} because no QGLViewer is available")
  return()
endif()


if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)


set(SRC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src)


set(APP_INCLUDES 
  ${SRC} 
  ${CORE_INCLUDES} 
  ${OPENGL_INCLUDE_DIR} 
  ${QGLVIEWER_INCLUDE_DIRS} 
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Xml
  Qt5::PrintSupport)

include_directories(${APP_INCLUDES})

set(APP_LIBRARIES 
  qlidarview-gui
  ${QGLVIEWER_LIBRARY}
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Multimedia
  Qt5::MultimediaWidgets 
  Qt5::Xml
  Qt5::PrintSupport
  ${OPENGL_LIBRARIES}
  ${CORE_LIBRARIES})

#  v4lconvert
#  v4l2

file(GLOB_RECURSE SOURCE_FILES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp)

file(GLOB_RECURSE HEADER_FILES
	${SRC}/*.h
	${SRC}/*.hpp)

if( EXISTS ${SRC}/app-resources.qrc )
	qt5_add_resources(RESOURCE_FILES
		${SRC}/app-resources.qrc)
endif()



add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
	${APP_LIBRARIES})


# Install

set(MY_RUNTIME_PATH bin)
set(MY_DESKTOP_ENTRY_PATH ${CMAKE_INSTALL_PREFIX}/share/applications)
set(MY_DESKTOP_ENTRY_COMMENT "${PROJECT_NAME}")
set(MY_APP_ICONS_PATH ${CMAKE_INSTALL_PREFIX}/share/app-install/icons)

if( EXISTS ${SRC}/app.desktop.in)
	configure_file(${SRC}/app.desktop.in
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop
		@ONLY)
endif()

if(EXISTS ${SRC}/app-icon.png)
	configure_file(${SRC}/app-icon.png
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png
		COPYONLY)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME
	DESTINATION ${MY_RUNTIME_PATH})

if( EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop )
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop
		DESTINATION ${MY_DESKTOP_ENTRY_PATH})
endif()

if( EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png )
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png
		DESTINATION ${MY_APP_ICONS_PATH})
endif()

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

