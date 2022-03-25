CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(qlidarview-core CXX)

# May be need for python bindings ?
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# target sources
file(GLOB_RECURSE SOURCE_FILES 
  "*.c"
  "*.cc"
  "*.cpp")
 
file(GLOB_RECURSE HEADER_FILES
  "*.h"
  "*.hpp")

add_library(${PROJECT_NAME} STATIC
	${HEADER_FILES}
	${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CORE_INCLUDES} )

# remove duplicates from include direcories
get_property(__incdirs TARGET ${PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES)
list(REMOVE_DUPLICATES __incdirs)
set_property( TARGET ${PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES ${__incdirs})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
foreach ( file ${HEADER_FILES} )
   	get_filename_component( dir ${file} DIRECTORY )
    file(RELATIVE_PATH relpath "${CMAKE_CURRENT_SOURCE_DIR}" ${dir})
    install( FILES ${file} DESTINATION include/qlidarview/core/${relpath} )
endforeach()


