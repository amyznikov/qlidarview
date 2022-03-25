cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(qlidarview C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-march=native)
if( ENABLE_CALL_GRAPH_INSTRUMENTATION )
	add_definitions(-DENABLE_CALL_GRAPH_INSTRUMENTATION=1)
endif()


set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# OpenCV
set(OpenCV_MINIMUM_VERSION_REQUIRED "4.2")
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc imgcodecs features2d flann video videoio objdetect xobjdetect OPTIONAL_COMPONENTS xfeatures2d) # freetype
if((OpenCV_VERSION VERSION_LESS ${OpenCV_MINIMUM_VERSION_REQUIRED}) )
   message(FATAL_ERROR "ERROR: OpenCV version >= ${OpenCV_MINIMUM_VERSION_REQUIRED} is required but '${OpenCV_VERSION}' is found")
   return()
endif()
message(STATUS "OpenCV:  ${OpenCV_VERSION}")
message(STATUS "xfeatures2d:  ${OpenCV_xfeatures2d_FOUND}")
if ( OpenCV_xfeatures2d_FOUND ) 
	add_definitions(-DHAVE_xfeatures2d=1)
endif()	



# TIFF : TIFF_LIBRARIES TIFF_INCLUDE_DIR
find_package(TIFF)
if ( NOT TIFF_FOUND )
  message( FATAL_ERROR "TIFF package is mandatory but is not found in your system. Install libtiff package please.")
endif()
message(STATUS "tiff:  ${TIFF_INCLUDE_DIR} ${TIFF_LIBRARIES}")


# tinyxml2
find_package(tinyxml2)
if ( NOT tinyxml2_FOUND )
  message( FATAL_ERROR "tinyxml2 package is mandatory but is not found in your system. Install tinyxml2 package please.")
endif()
get_target_property(TINYXML2_INCLUDE_DIR tinyxml2::tinyxml2 INTERFACE_INCLUDE_DIRECTORIES)
set(TINYXML2_LIBRARIES tinyxml2::tinyxml2)
message(STATUS "tinyxml2: ${tinyxml2_VERSION} ${TINYXML2_INCLUDE_DIR} ${TINYXML2_LIBRARIES}")


# PCAP
unset(PCAP_INCLUDE_DIR)
unset(PCAP_LIBRARY)
find_path(PCAP_INCLUDE_DIR pcap/pcap.h)
find_library(PCAP_LIBRARY pcap)

if( PCAP_INCLUDE_DIR AND PCAP_LIBRARY ) 
    message(STATUS "pcap: ${PCAP_INCLUDE_DIR} ${PCAP_LIBRARY}")
    add_definitions(-DHAVE_PCAP=1)
else()
	set(PCAP_INCLUDE_DIR "")
	set(PCAP_LIBRARY "")
	
    message(WARNING "Can not locate pcap library under search path. 
    	pcap file reading functionalitymay not work.
    	Install please pcap package")
    	
endif()




# Intel TBB
if ( TRUE )
find_package(TBB QUIET)
if (TBB_FOUND)

  get_target_property(TBB_INCLUDES TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
  if ( NOT TBB_INCLUDES ) # This happend under SUSE: TBB_FOUND is TRUE but INTERFACE_INCLUDE_DIRECTORIES is not set
        SET(TBB_INCLUDES /usr/include)
        MESSAGE(WARNING, "INTERFACE_INCLUDE_DIRECTORIES was not set by TBBConfig.cmake, force TBB_INCLUDES to ${TBB_INCLUDES}")
  endif()
  
  set(TBB_LIBRARIES TBB::tbb)

elseif(UNIX)
  # It seems we are working on old version of ubuntu or apple, try to locate files manually

  find_path(TBB_INCLUDES tbb/tbb.h
    /usr/local/include/
    /opt/local/include/
    /usr/include)

  if ( NOT TBB_INCLUDES )
    message(FATAL_ERROR "Can not locate TBB include directory under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)")
  endif()

  find_library(TBB_LIBRARIES
    NAMES tbb 
    PATHS /usr/local/lib
          /opt/local/lib
          /usr/lib
          ENV LD_LIBRARY_PATH
          ENV LIBRARY_PATH)

  if ( NOT TBB_LIBRARIES )
    message(FATAL_ERROR "Can not locate TBB library under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)")
  endif()

  set(TBB_FOUND TRUE)
  
else()
    message(FATAL_ERROR "Can not locate TBB library under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)") 
endif()

add_definitions(-DHAVE_TBB=1 -DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)
message (STATUS "TBB: ${TBB_INCLUDES} ${TBB_LIBRARIES}")
endif() # TBB


# libconfig
if ( FALSE )
set(LIBCONFIG_MINIMUM_VERSION_REQUIRED "1.7")
find_package(libconfig)
message(STATUS "libconfig [ ${LIBCONFIG_VERSION}, ${libconfig_VERSION}] found=${libconfig_FOUND}")
if ( NOT ${libconfig_FOUND} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is not found\n \
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()
if ( libconfig_VERSION AND NOT LIBCONFIG_VERSION ) # Th case of the lellers vary between ubuntu and arch linux
    set(LIBCONFIG_VERSION ${libconfig_VERSION})
endif()
if ( NOT LIBCONFIG_VERSION )
    message(FATAL_ERROR "Can not determine libconfig version, seems libconfig cmake skript is broken.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()
if ( LIBCONFIG_VERSION VERSION_LESS ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is required but libconfig version ${LIBCONFIG_VERSION} found.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()

if ( NOT LIBCONFIG_INCLUDE_DIRS )
  if (WIN32)
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::libconfig INTERFACE_INCLUDE_DIRECTORIES)
  else()
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::config INTERFACE_INCLUDE_DIRECTORIES)
  endif()
endif()

if ( NOT LIBCONFIG_LIBRARIES ) 
  if (WIN32)
    set(LIBCONFIG_LIBRARIES libconfig::libconfig)
  else() 
    set(LIBCONFIG_LIBRARIES libconfig::config)
  endif()
endif()
message (STATUS "libconfig : ${LIBCONFIG_VERSION} ${LIBCONFIG_INCLUDE_DIRS} ${LIBCONFIG_LIBRARIES}")
endif() # libconfig


# QGLViewer

set(HAVE_QGLViewer FALSE)  
find_package(QGLViewer QUIET)
if ( NOT QGLViewer_FOUND )

	set(QGLVIEWER_INCLUDE_DIRS "")
	set(QGLVIEWER_LIBRARY "")

    message(WARNING "QGLViewer >= 2.7 is required for 3D clouds browsing. \n \
        Install please fresh version of QGLViewer from AUR or from sources: \n \
        \n \
         wget http://www.libqglviewer.com/src/libQGLViewer-2.7.1.tar.gz && \\ \n \
         tar -xzvf libQGLViewer-2.7.1.tar.gz && \\ \n \
         cd libQGLViewer-2.7.1/QGLViewer && \\ \n \
         qmake && \\ \n \
         make && \\ \n \
         sudo make install && \\ \n \
         make clean\ \n \
     ")

else()
	# OpenGL is required for QGLViewer
	# This option should affect only OpenGL-dependent targets
	# In case of multiple choices prefer vendor OPENGL_gl_LIBRARY by default
	# (usually libOpenGL.so and libGLX.so).
	if( ${CMAKE_VERSION} VERSION_GREATER "3.10" AND NOT OpenGL_GL_PREFERENCE )
	    set(OpenGL_GL_PREFERENCE GLVND)
	endif()
	
	find_package(OpenGL QUIET)
	if ( NOT OpenGL_FOUND )
        message(WARNING "OpenGL package is not found, disabling QGLViewer")
        set(QGLVIEWER_INCLUDE_DIRS)
        set(QGLVIEWER_LIBRARY)
        set(OPENGL_INCLUDE_DIR)
        set(OPENGL_LIBRARY_DIR)
        set(OPENGL_LIBRARIES)
        set(QGLViewer_FOUND FALSE)
    else ()
        find_file(freeglut_h_found GL/freeglut.h  /usr/local/include /usr/include)
        if ( NOT freeglut_h_found ) 
            message(WARNING "OpenGL package is not found, disabling QGLViewer")
            set(QGLVIEWER_INCLUDE_DIRS)
            set(QGLVIEWER_LIBRARY)
            set(OPENGL_INCLUDE_DIR)
            set(OPENGL_LIBRARY_DIR)
            set(OPENGL_LIBRARIES)
            set(QGLViewer_FOUND FALSE)
        else()
            set(HAVE_QGLViewer TRUE)
            add_definitions(-DHAVE_QGLViewer=1)

            # Fixing GLEW PATH on OSX
            if (APPLE)
                find_path( GLEW_INCLUDE_DIR GL/glew.h
                    /usr/local/include
                    /usr/local/Cellar/glew/2.1.0/include
                    /System/Library/Frameworks/GLEW.framework/Versions/A/Headers
                    ${OPENGL_LIBRARY_DIR}
                )
                find_library( GLEW_GLEW_LIBRARY GLEW
                    /usr/local/lib
                    /usr/openwin/lib
                    /usr/X11R6/lib
                    /usr/local/Cellar/glew/2.1.0/lib
                )
                set( GLEW_FOUND TRUE )
                if(NOT GLEW_GLEW_LIBRARY)
                    set(GLEW_GLEW_LIBRARY "-framework GLEW" CACHE STRING "GLEW library for OSX")
                    set(GLEW_cocoa_LIBRARY "-framework Cocoa" CACHE STRING "Cocoa framework for OSX")
                    set(GLEW_FOUND FALSE )
                endif()
            endif (APPLE)
        endif()
    endif()
endif()


# Qt5
if ( QGLViewer_FOUND )
	set(Qt5_REQUIRED_COMPONENTS Core Gui Widgets Xml PrintSupport Multimedia MultimediaWidgets OpenGL) 

	find_package(Qt5 5.10 COMPONENTS ${Qt5_REQUIRED_COMPONENTS}) 
	foreach ( module ${Qt5_REQUIRED_MODULES} )
	    find_package(${module} ${Qt5_MINIMUM_VERSION_REQUIRED})
	    if ( NOT ${module}_FOUND )
	        message( FATAL_ERROR "${module} ${Qt5_MINIMUM_VERSION_REQUIRED} NOT FOUND")
	    endif()
	endforeach()

endif()

set(CORE_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}
#	${LIBCONFIG_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${TIFF_INCLUDE_DIR}
	${TBB_INCLUDES}
	${PCAP_INCLUDE_DIR}
	${TINYXML2_INCLUDE_DIR}
    )

set(CORE_LIBRARIES
	qlidarview-core
#	${LIBCONFIG_LIBRARIES}
	${OpenCV_LIBS}
	${TIFF_LIBRARIES}
    ${TBB_LIBRARIES}
    ${PCAP_LIBRARY}
    ${TINYXML2_LIBRARIES})



# Walk over subdirs
add_subdirectory(core)
add_subdirectory(gui)
add_subdirectory(apps)

# pseudo-target for building libraries only
add_custom_target(libs)
add_dependencies(libs qlidarview-core)
add_dependencies(libs qlidarview-gui)


add_custom_target(uninstall
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
