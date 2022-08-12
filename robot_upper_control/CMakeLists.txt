##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(robot_upper_control)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#添加c++11的支持
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#CMAKE_CURRENT_SOURCE_DIR：当前正在处理的源目录的路径
#CMAKE_SOURCE_DIR： 最外层CMakeLists.txt所在目录, 这里会找到learning_communication/
#PROJECT_SOURCE_DIR为包含PROJECT()的最近一个CMakeLists.txt文件所在的文件夹, 这里为robot_upper_control/

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    rviz
    )

# 该语句会定义变量CATKIN_PACKAGE_LIB_DESTINATION  CATKIN_PACKAGE_BIN_DESTINATION, 会用于设置输出等目录
catkin_package(
)

##############################################################################
# Qt
##############################################################################

#开启MOC、RCC和UIC，则会在预处理之前先进行MOC、RCC和UIC操作
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
#设置UIC的查找路径，UIC默认会在引用ui头文件的源代码路径下查找ui文件，而我们的ui文件存放在/ui文件夹下
set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/ui)

# CMAKE_PREFIX_PATH供find_package()使用
set(CMAKE_PREFIX_PATH "/opt/Qt/Qt5.15.2/5.15.2/gcc_64")
# ind_package()时需要提供模块的*.cmake文件
set(Qt5_DIR "${CMAKE_PREFIX_PATH}/lib/cmake/Qt5")

#查找需要的Qt包
find_package(Qt5 REQUIRED COMPONENTS
    Widgets
    Core
    Gui
    )
set(QT_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Gui)
message("this is Qt5_DIR ${Qt5_DIR}")
message("this is QT_LIBRARIES ${QT_LIBRARIES}")
message("this is CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}")

##############################################################################
# Sections
##############################################################################

#将各个文件夹下的所有文件打包
FILE(GLOB ${PROJECT_NAME}_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")
#FILE(GLOB ${PROJECT_NAME}_QRC "${PROJECT_SOURCE_DIR}/qrc/*.qrc")
FILE(GLOB_RECURSE ${PROJECT_NAME}_HEADER FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.h *.hpp)

#直接将上述的所有文件进行编译和内部链接，ui和qrc以及包含Qt扩展语法的文件由于开启了开启MOC、RCC和UIC，因此会被自行处理
add_executable(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_SOURCES}
#    ${${PROJECT_NAME}_QRC}
    ${${PROJECT_NAME}_HEADER}
    )

#将/include路径包含到查找头文件路径
include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${Qt5_INCLUDE_DIRS}
    ${wxWidgets_USE_FILE}
    )

#链接
target_link_libraries(
    ${PROJECT_NAME}_node ${catkin_LIBRARIES} ${QT_LIBRARIES}
    )

#############################################################################################
# install
#############################################################################################

#设置可执行文件的输出路径，默认在哪个路径编译则会放在哪个路径，而我们是在/build下编译，要存放在/bin路径
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME}/)
#install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#添加的支持debug和release支持
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
