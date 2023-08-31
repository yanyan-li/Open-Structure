cmake_minimum_required(VERSION 3.10)
project(Venom)

SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -Wno-unused-function -Wno-unused-private-field -Wno-unused-label")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall --warn-unused-variable  --warn-unused-parameter  ")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -warn-unused-variable -warn-unused-parameter -warn-unused-function -warn-unused-private-field -Wno-unused-label")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_STANDARD 17) 

# installed in docker already 
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

option(VERBOSE "Venom Similator Printer" ON)
option(DEBUG "Venom Similator Debug" ON)
if(VERBOSE)
 message(STATUS "---Open Venom Simulator Printer---")
 add_definitions(-D__VERBOSE__)
endif()
if(DEBUG)
 message(STATUS "---Open Venom Simulator Debug---")
 add_definitions(-D__DEBUG__)
endif()

# installed later if you need
find_package(Ceres REQUIRED)  # required version-1.14

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}/eigen3
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
)
add_subdirectory(thirdparty)

file(GLOB SOURCEFILES ${PROJECT_SOURCE_DIR}/src/*/*.hpp ${PROJECT_SOURCE_DIR}/src/*/*/*.cc)
#file(GLOB SOURCEFILES src/*.hpp)

add_library(VenomLib SHARED ${SOURCEFILES})
target_link_libraries(VenomLib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES})
        #gtsam)


#########---simulation interface---###################
add_executable(start_venom examples/main_entrance.cc)
target_link_libraries(start_venom VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

add_executable(test_draw_corridor examples/test_draw_corridor.cc)
target_link_libraries(test_draw_corridor VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

add_executable(test_draw_corridor_trajectory examples/test_draw_cooridor_trajectory.cc)
target_link_libraries(test_draw_corridor_trajectory VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

add_executable(test_draw_corridor_envpoint examples/test_draw_corridor_envpoint.cc)
target_link_libraries(test_draw_corridor_envpoint VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

add_executable(test_draw_corridor_envpl examples/test_draw_corrifor_envpointline.cc)
target_link_libraries(test_draw_corridor_envpl VenomLib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_draw_room_envpl examples/test_draw_room_envpointline.cc)
target_link_libraries(test_draw_room_envpl VenomLib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_draw_box_envpl examples/test_draw_objects_envpointline.cc)
target_link_libraries(test_draw_box_envpl VenomLib ${CMAKE_THREAD_LIBS_INIT})


add_executable(test_draw_new_line examples/test_line_rect_intersection.cc)
target_link_libraries(test_draw_new_line VenomLib ${CMAKE_THREAD_LIBS_INIT})

