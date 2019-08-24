INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()



add_executable(aruco_ros aruco_ros.cpp)


target_link_libraries(aruco_ros                aruco opencv_calib3d ${THREADLIB})

#aruco_create_board
INSTALL(TARGETS aruco_ros    RUNTIME DESTINATION bin)
