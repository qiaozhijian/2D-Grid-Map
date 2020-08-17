CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( octomap_robot )


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

SET(CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE Release  )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_CXX_FLAGS -std=c++11)

FIND_PACKAGE( PCL REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)

# octomap
set(OCTOMAP_INCLUDE_DIRS "/media/qzj/Software/code/octomap/octomap/include")
set(OCTOMAP_LIBRARY_DIRS "/media/qzj/Software/code/octomap/lib")


# Set library names as absolute paths:
set(OCTOMAP_LIBRARIES
        "/media/qzj/Software/code/octomap/lib/liboctomap.so"
        "/media/qzj/Software/code/octomap/lib/liboctomath.so"
        )

INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src
)

add_library(my_util SHARED
        src/handlePCL.cpp
        )
target_link_libraries(my_util
        ${PCL_LIBRARIES}
        )

ADD_EXECUTABLE( pcd2octomap pcd2octomap.cpp )
TARGET_LINK_LIBRARIES( pcd2octomap
        my_util
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE( pcd2colorOctomap pcd2colorOctomap.cpp )
TARGET_LINK_LIBRARIES( pcd2colorOctomap
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES})

FIND_PACKAGE(OpenCV REQUIRED)

ADD_EXECUTABLE( joinmap joinMap.cpp )
TARGET_LINK_LIBRARIES( joinmap
        ${OCTOMAP_LIBRARIES}
        ${OpenCV_LIBS})

