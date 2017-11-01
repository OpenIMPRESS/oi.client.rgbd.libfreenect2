CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()


IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(oi.client.rgbd.libfreenect2)
SET(CMAKE_CXX_STANDARD 11)

SET(freenect2_ROOT_DIR ${MY_DIR}/../libfreenect2 CACHE STRING "Root folder of libfreenect2 source.")
SET(asio_ROOT_DIR "${MY_DIR}/../asio" CACHE STRING "Root folder of asio source.")

SET(MY_DIR ${PROJECT_SOURCE_DIR})
SET(DEPENDS_DIR "${MY_DIR}${freenect2_ROOT_DIR}/depends" CACHE STRING "libfreenect2 Dependency directory")

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)
LIST(APPEND CMAKE_MODULE_PATH "${freenect2_ROOT_DIR}/cmake_modules" ) # freenect2 Modules
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ) # oi.client.rgbd.libfreenect2 Modules

FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(ASIO REQUIRED)
FIND_PACKAGE(TurboJPEG REQUIRED)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )

INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})    
INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/headers)      
INCLUDE_DIRECTORIES(${TurboJPEG_INCLUDE_DIRS})


SET(application_src ${PROJECT_SOURCE_DIR}/src/main.cpp)

if(WIN32)
  SET(application_LIBRARIES optimized ${freenect2_LIBRARIES} debug ${freenect2_LIBRARIES_D})
  SET(application_DLLS optimized ${freenect2_DLLS} debug ${freenect2_DLLS_D})
ELSE()
  SET(application_LIBRARIES ${freenect2_LIBRARIES})
  SET(application_DLLS ${freenect2_DLLS})
ENDIF()

LIST(APPEND application_LIBRARIES ${TurboJPEG_LIBRARIES})

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LIST(APPEND application_headers
      ${PROJECT_SOURCE_DIR}/headers/config.h
      ${PROJECT_SOURCE_DIR}/headers/json.hpp
      ${PROJECT_SOURCE_DIR}/headers/stb_dxt.h
      ${PROJECT_SOURCE_DIR}/headers/viewer.h
    )
    LIST(APPEND application_DLLS ${GLFW3_DLL})
    LIST(APPEND application_src
      ${PROJECT_SOURCE_DIR}/src/oi.plugin.network.cpp
      ${PROJECT_SOURCE_DIR}/src/viewer.cpp
      ${flextGL_SOURCES}
    )
    LIST(APPEND application_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )

    ADD_DEFINITIONS(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

SOURCE_GROUP("Header Files" FILES ${application_headers})

ADD_EXECUTABLE(oi.client.rgbd.libfreenect2
  ${application_src}
  ${application_headers}
)

TARGET_LINK_LIBRARIES(oi.client.rgbd.libfreenect2
  ${application_LIBRARIES}
)

#SET(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations")
