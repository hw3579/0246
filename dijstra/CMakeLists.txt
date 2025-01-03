# 设置CMake的最低版本
cmake_minimum_required(VERSION 3.10)

set (CMAKE_CXX_STANDARD 17)

# 设置项目名称
project(MyQtApp)

# 设置OpenCV路径
if(MSVC)
	set(OpenCV_DIR "C:/tools/opencv/build")
elseif(UNIX)
	set(OpenCV_DIR "/usr/local/opencv/build")
endif()

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 包含 OpenCV 的头文件目录
include_directories(include ${OpenCV_INCLUDE_DIRS})

# 添加源文件
set(SOURCES
    src/dijkstra_eigen.cpp
    src/random_map_eigen.cpp
    src/point.cpp
    src/main.cpp
    src/vector2mat.cpp
)
# 设置可执行文件输出
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接 OpenCV 库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})