cmake_minimum_required(VERSION 3.10)

set (CMAKE_CXX_STANDARD 17)

project(Dijkstra)

# set oepncv path
if(MSVC)
	set(OpenCV_DIR "C:/tools/opencv/build") # change here if windows have different path
elseif(UNIX)
	set(OpenCV_DIR "/usr/local/opencv/build")
endif()

# find OpenCV
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

set(SOURCES
    src/dijkstra_eigen.cpp
    src/random_map_eigen.cpp
    src/point.cpp
    
)

add_executable(${PROJECT_NAME} ${SOURCES} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(c-space ${SOURCES} src/c_space.cpp)
target_link_libraries(c-space ${OpenCV_LIBS})