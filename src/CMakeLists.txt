# 增加PCL库的依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
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

ADD_EXECUTABLE( pcd2octomap pcd2octomap.cpp )
TARGET_LINK_LIBRARIES( pcd2octomap
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

