project(oi.client.rgbd.libfreenect2 VERSION 0.1 LANGUAGES CXX)

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

LIST(APPEND CMAKE_MODULE_PATH ${OI_NATIVE_MODULE_PATH} ) 
LIST(APPEND CMAKE_MODULE_PATH ${OI_DEPENDS_DIR}/libfreenect2/cmake_modules ) 

FIND_PACKAGE(ASIO REQUIRED)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(TurboJPEG REQUIRED)

file(GLOB SRC "src/*.cpp")
file(GLOB HEADERS "include/*.hpp" "src/*.hpp")
add_executable(${PROJECT_NAME} ${SRC} ${HEADERS})
source_group("Header Files" FILES ${HEADERS})
target_link_libraries(${PROJECT_NAME} oi.core oi.network oi.rgbd)
target_link_libraries(${PROJECT_NAME} ${freenect2_LIBRARIES} ${TurboJPEG_LIBRARIES})


target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

INCLUDE_DIRECTORIES(${OI_LIB_DIR})
INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})    
INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${TurboJPEG_INCLUDE_DIRS})

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES( ${GLFW3_INCLUDE_DIRS} ${flextGL_INCLUDE_DIRS} )
    #LIST(APPEND application_DLLS ${GLFW3_DLL} )
    #LIST(APPEND application_SRC ${flextGL_SOURCES} )
    #LIST(APPEND application_LIBRARIES ${GLFW3_LIBRARIES} ${OPENGL_gl_LIBRARY} )
    target_link_libraries(${PROJECT_NAME} ${GLFW3_LIBRARIES} ${OPENGL_gl_LIBRARY} )
  ELSE()
      MESSAGE("ENABLE_OPENGL was set, but could not find GLFW3 and/or OpenGL")
  ENDIF()
ENDIF(ENABLE_OPENGL)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${PROJECT_NAME} EXPORT OIClientLibfreenect2Config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT OIClientLibfreenect2Config DESTINATION share/OIRGBD/cmake)

# This makes the project importable from the build directory
export(TARGETS ${PROJECT_NAME} FILE OIClientLibfreenect2Config.cmake)
