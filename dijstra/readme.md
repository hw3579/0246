# Dijkstra Path Planning for coursework

This is a project for path planning and visualization using the Dijkstra algorithm.

## Dependencies

- **C++11** or later
- **OpenCV 4.x**
- **CMake 3.10** or later

## Compilation and Execution

### 1. Clone the Repository

```bash
git clone https://github.com/your-repo/path-planning.git
cd path-planning
```

### 2. Create Build Directory and Run CMake

```bash
mkdir build
cd build
cmake ..
```

### 3. Compile the Project

```bash
make
```

### 4. Run the Executable

Path Planning
```bash
./path_planning
```
Configuration Space
```bash
./c-space
```
## File Structure

```plaintext
src/: Source code directory
  - main.cpp: Entry point of the program
  - random_map_eigen.h and random_map_eigen.cpp: map generation
  - dijkstra_eigen.h and dijkstra_eigen.cpp: Dijkstra algorithm implementation
  - main.h: Additional helper functions and definitions
  - c_space.cpp and c_space.h: Configuration space implementation

CMakeLists.txt: CMake build script
```

## Example

After running the program, a visualized map containing obstacles and the planned path will be generated.

The save path is in the build directory and in **/img** folder.

## Configuration Space
The c-space executable is used to construct the configuration space. In the main.cpp file, the macro definition #define is_include_tank false is used to determine whether to include the tank volume in the calculations.

