CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

project(dump_hdl_lidar_specifcation_xml CXX)

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

file(GLOB_RECURSE SOURCE_FILES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp)

file(GLOB_RECURSE HEADER_FILES
	${SRC}/*.h
	${SRC}/*.hpp)

set(APP_INCLUDES
	${SRC}
    ${CORE_INCLUDES})

set(APP_LIBRARIES 
    ${CORE_LIBRARIES})

add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})


include_directories(
    ${APP_INCLUDES})

target_link_libraries(${PROJECT_NAME}
	${APP_LIBRARIES})

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

