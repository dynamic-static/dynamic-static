
# dynamic-static

TODO : More README

# Getting started

Ensure the following tools are installed...
 - [CMake](https://cmake.org/download/) v3.3+ (Make sure to select "Add to PATH" when prompted)
 - [Git](https://git-scm.com/)
 - [Python](https://www.python.org/downloads/) v3+ (Make sure to select "Add to PATH" when prompted)
 - [Visual Studio](https://visualstudio.microsoft.com/vs/community/) 2019 (Make sure to select "Desktop development with C++" when prompted)
 - [Vulkan SDK](https://vulkan.lunarg.com/sdk/home) v1.3.216.0+

The following instructions are for a  `bash` like terminal (Git Bash comes with the Git install by default on Windows)...
```
cd c:
cd <desired/directory/location/>
git clone https://github.com/dynamic-static/dynamic-static.git
cd dynamic-static/
mkdir build
cd build/
cmake -G "Visual Studio 16 2019" -A x64 ..
cmake --build .
```
