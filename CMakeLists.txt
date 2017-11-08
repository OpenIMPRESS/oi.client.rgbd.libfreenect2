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

SET(MY_DIR ${PROJECT_SOURCE_DIR})
SET(freenect2_ROOT_DIR "${MY_DIR}/../../dependencies/libfreenect2" CACHE STRING "Root folder of libfreenect2 source.")
SET(ASIO_DIR "${MY_DIR}/../../dependencies/asio" CACHE STRING "Root folder of asio source.")

SET(DEPENDS_DIR "${freenect2_ROOT_DIR}/depends" CACHE STRING "libfreenect2 Dependency directory")

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h
FIND_PACKAGE(PkgConfig)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ) # oi.client.rgbd.libfreenect2 Modules
LIST(APPEND CMAKE_MODULE_PATH "${freenect2_ROOT_DIR}/cmake_modules" ) # freenect2 Modules

FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(ASIO REQUIRED)
FIND_PACKAGE(TurboJPEG REQUIRED)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )

INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})    
INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/headers)      
INCLUDE_DIRECTORIES(${TurboJPEG_INCLUDE_DIRS})


SET(application_SRC ${PROJECT_SOURCE_DIR}/src/main.cpp)

IF(WIN32) # We're just copying all .dlls over from libfreenect2
  GET_FILENAME_COMPONENT(path_RELEASE ${freenect2_DLL} DIRECTORY )
  GET_FILENAME_COMPONENT(path_DEBUG ${freenect2_DLL_D} DIRECTORY )
  FILE(GLOB dlls_RELEASE "${path_RELEASE}/*.dll")
  FILE(GLOB dlls_DEBUG "${path_DEBUG}/*.dll")
  FILE(COPY ${dlls_RELEASE} DESTINATION ${CMAKE_BINARY_DIR}/Release)
  FILE(COPY ${dlls_DEBUG} DESTINATION ${CMAKE_BINARY_DIR}/Debug)
  SET(application_LIBRARIES optimized ${freenect2_LIBRARIES} debug ${freenect2_LIBRARIES_D})
  SET(application_DLLS optimized ${freenect2_DLLS} debug ${freenect2_DLLS_D})
  SET(freenect2_DLL ${CMAKE_BINARY_DIR}/Release/freenect2.dll)
  SET(freenect2_DLL ${CMAKE_BINARY_DIR}/Debug/freenect2d.dll)
  SET(freenect2_DLLS ${freenect2_DLL})
  SET(freenect2_DLLS_D ${freenect2_DLL_D})
ELSE()
  SET(application_LIBRARIES ${freenect2_LIBRARIES})
  SET(application_DLLS ${freenect2_DLLS})
ENDIF()

LIST(APPEND application_LIBRARIES ${TurboJPEG_LIBRARIES})

LIST(APPEND application_HEADERS
  ${PROJECT_SOURCE_DIR}/headers/config.h
  ${PROJECT_SOURCE_DIR}/headers/json.hpp
  ${PROJECT_SOURCE_DIR}/headers/stb_dxt.h
  ${PROJECT_SOURCE_DIR}/headers/UDPConnector.hpp
  ${PROJECT_SOURCE_DIR}/headers/RGBDStreamer.hpp
  ${PROJECT_SOURCE_DIR}/headers/LibFreenect2Streamer.hpp
)

LIST(APPEND application_SRC
  ${PROJECT_SOURCE_DIR}/src/UDPConnector.cpp
  ${PROJECT_SOURCE_DIR}/src/RGBDStreamer.cpp
  ${PROJECT_SOURCE_DIR}/src/LibFreenect2Streamer.cpp
)

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LIST(APPEND application_DLLS ${GLFW3_DLL})

    LIST(APPEND application_SRC
      ${flextGL_SOURCES}
    )

    LIST(APPEND application_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
  ELSE()
      MESSAGE("ENABLE_OPENGL was set, but could not find GLFW3 and/or OpenGL")
  ENDIF()
ENDIF(ENABLE_OPENGL)

SOURCE_GROUP("Header Files" FILES ${application_HEADERS})

ADD_EXECUTABLE(oi.client.rgbd.libfreenect2
    ${application_SRC}
    ${application_HEADERS}
)

TARGET_LINK_LIBRARIES(oi.client.rgbd.libfreenect2
  ${application_LIBRARIES}
)

#SET(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations")
