
# dynamic-static

TODO : More README

# Getting Started

Ensure the following tools are installed...
 - [CMake](https://cmake.org/download/) v3.3+ (Make sure to select "Add to PATH" when prompted)
 - [Git](https://git-scm.com/)
 - [Python](https://www.python.org/downloads/) v3+ (Make sure to select "Add to PATH" when prompted)
 - [Visual Studio](https://visualstudio.microsoft.com/vs/community/) 2019 (Make sure to select "Desktop development with C++" when prompted)
 - [Vulkan SDK](https://vulkan.lunarg.com/sdk/home) v1.3.239.0

The following command lines are for configuring a Visual Studio solution using a  `bash` like terminal (Git Bash comes with the Git install by default on Windows) in a directory called `gitrepos/dynamic-static` (you can use any directory you'd like) on drive `C:`...
```
cd c:
cd gitrepos/dynamic-static
git clone https://github.com/dynamic-static/dynamic-static.git
cd dynamic-static/
cmake -G "Visual Studio 16 2019" -A x64 -B ./build .
cmake --build ./build
```
Open `gitrepos/dynamic-static/dynamic-static/build/dynamic-static.sln` in Visual Studio, navigate to `samples/brick-breaker`, right click and select "Set as Startup Project", run.
