cmake_minimum_required(VERSION 2.8.0)
project(ur_assemble_ui)

add_compile_options(-std=c++11)

set(Qt5Widgets_DIR ~/Qt5.9.0/5.9/gcc_64/lib/cmake/Qt5Widgets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  qt_build
  ur_control_msgs
  cmake_modules
)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_RESOURCES
  resources/images.qrc
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ur_control_msgs
  DEPENDS system_lib
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  include
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_MOCS} ${${PROJECT_NAME}_UIS_H} ${${PROJECT_NAME}_RCC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resources ui launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)