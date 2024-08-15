cmake_minimum_required(VERSION 3.5)
project(attitude_prediction)

# 找到依赖包
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
)

find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 包含头文件目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

file(GLOB_RECURSE MAIN_SOURCES "src/*.cpp")

# 编译源文件生成库
add_library(${PROJECT_NAME} SHARED ${MAIN_SOURCES})

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 编译测试用例
add_executable(test_orb test/TestOrb.cpp ${MAIN_SOURCES})

ament_target_dependencies(test_orb
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS 
  test_orb
  DESTINATION lib/${PROJECT_NAME}
)

############

add_executable(test_vo test/TestVo.cpp ${MAIN_SOURCES})

ament_target_dependencies(test_vo
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS 
test_vo
  DESTINATION lib/${PROJECT_NAME}
)

############

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
