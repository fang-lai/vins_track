cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_models
    cv_bridge
    roslib
    )

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

set(Protobuf_PREFIX_PATH
    ${PROJECT_SOURCE_DIR}/support_files/protobuf-3.6.0/include
    ${PROJECT_SOURCE_DIR}/support_files/protobuf-3.6.0/lib
    ${PROJECT_SOURCE_DIR}/support_files/protobuf-3.6.0/bin
)

list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")
find_package(Protobuf REQUIRED)
# include_directories(${CMAKE_CURRENT_BINARY_DIR})
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS "src/utility/database.proto") #(无法使用)
# PROTOBUF_GENERATE_CPP默认生成的文件放在build目录下，复制出来便于其他项目文件引用 (无法复制)
# add_custom_target(CopyTask COMMAND ${CMAKE_COMMAND} -E copy  ${PROTO_HDRS}  "/home/jon/code/vins_ws/src/VINS-Fusion/loop_fusion/src/utility/database.pb.h" )
message(STATUS "+++ protobuf versiion:  ${Protobuf_VERSION}")

include_directories(${catkin_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIR} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} )

catkin_package()

#protoc ./database.proto --cpp_out=./
add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    # ${PROTO_SRCS} 
    src/utility/database.pb.cc
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES} ${Protobuf_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ) 
