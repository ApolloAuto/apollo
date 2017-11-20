cmake_minimum_required(VERSION 2.8.3)
project(camera)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)

set(VIBRANTE TRUE)
add_definitions(-DVIBRANTE)

# cuda
set(cuda_INCLUDE_DIR /usr/local/cuda/include)
set(cuda_LIBRARY /usr/local/cuda/lib64)
link_directories(${cuda_LIBRARY})
# driveworks
set(driveworks_INCLUDE_DIR /usr/local/driveworks/include)
set(driveworks_LIBRARY /usr/local/driveworks/lib)
link_directories(${driveworks_LIBRARY})
# glfw
set(glfw_INCLUDE_DIR /usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/include)
set(glfw_LIBRARY /usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/libs)
link_directories(${glfw_LIBRARY})

find_package(OpenCV REQUIRED)

catkin_package(LIBRARIES ${PROJECT_NAME})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${cuda_INCLUDE_DIR}
  ${driveworks_INCLUDE_DIR}
  ${glfw_INCLUDE_DIR}
  /usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/nvmedia
)

add_executable(front_camera 
  src/front_camera.cpp
  src/camera_px2.cpp
  src/WindowGLFW.cpp
  src/WindowEGL.cpp
  src/ProgramArguments.cpp
  src/ConsoleColor.cpp
)
target_link_libraries(front_camera 
  ${catkin_LIBRARIES}
  driveworks
  EGL
  nvmedia
  drm
  libglfw3.a
  ${cuda_LIBRARY}
  X11
  Xrandr
  Xi
  GLESv2
  Xxf86vm
)

add_executable(rear_camera 
  src/rear_camera.cpp
  src/camera_px2.cpp
  src/WindowGLFW.cpp
  src/WindowEGL.cpp
  src/ProgramArguments.cpp
  src/ConsoleColor.cpp
)
target_link_libraries(rear_camera 
  ${catkin_LIBRARIES}
  driveworks
  EGL
  nvmedia
  drm
  libglfw3.a
  ${cuda_LIBRARY}
  X11
  Xrandr
  Xi
  GLESv2
  Xxf86vm
)
