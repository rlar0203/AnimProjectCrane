# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build"

# Include any dependencies generated for this target.
include CMakeFiles/Crane.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Crane.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Crane.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Crane.dir/flags.make

CMakeFiles/Crane.dir/src/GLSL.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/GLSL.cpp.o: ../src/GLSL.cpp
CMakeFiles/Crane.dir/src/GLSL.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Crane.dir/src/GLSL.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/GLSL.cpp.o -MF CMakeFiles/Crane.dir/src/GLSL.cpp.o.d -o CMakeFiles/Crane.dir/src/GLSL.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/GLSL.cpp"

CMakeFiles/Crane.dir/src/GLSL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/GLSL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/GLSL.cpp" > CMakeFiles/Crane.dir/src/GLSL.cpp.i

CMakeFiles/Crane.dir/src/GLSL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/GLSL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/GLSL.cpp" -o CMakeFiles/Crane.dir/src/GLSL.cpp.s

CMakeFiles/Crane.dir/src/Link.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/Link.cpp.o: ../src/Link.cpp
CMakeFiles/Crane.dir/src/Link.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Crane.dir/src/Link.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/Link.cpp.o -MF CMakeFiles/Crane.dir/src/Link.cpp.o.d -o CMakeFiles/Crane.dir/src/Link.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Link.cpp"

CMakeFiles/Crane.dir/src/Link.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/Link.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Link.cpp" > CMakeFiles/Crane.dir/src/Link.cpp.i

CMakeFiles/Crane.dir/src/Link.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/Link.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Link.cpp" -o CMakeFiles/Crane.dir/src/Link.cpp.s

CMakeFiles/Crane.dir/src/MatrixStack.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/MatrixStack.cpp.o: ../src/MatrixStack.cpp
CMakeFiles/Crane.dir/src/MatrixStack.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Crane.dir/src/MatrixStack.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/MatrixStack.cpp.o -MF CMakeFiles/Crane.dir/src/MatrixStack.cpp.o.d -o CMakeFiles/Crane.dir/src/MatrixStack.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/MatrixStack.cpp"

CMakeFiles/Crane.dir/src/MatrixStack.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/MatrixStack.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/MatrixStack.cpp" > CMakeFiles/Crane.dir/src/MatrixStack.cpp.i

CMakeFiles/Crane.dir/src/MatrixStack.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/MatrixStack.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/MatrixStack.cpp" -o CMakeFiles/Crane.dir/src/MatrixStack.cpp.s

CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o: ../src/ObjectiveLink.cpp
CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o -MF CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o.d -o CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/ObjectiveLink.cpp"

CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/ObjectiveLink.cpp" > CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.i

CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/ObjectiveLink.cpp" -o CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.s

CMakeFiles/Crane.dir/src/Optimizer.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/Optimizer.cpp.o: ../src/Optimizer.cpp
CMakeFiles/Crane.dir/src/Optimizer.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Crane.dir/src/Optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/Optimizer.cpp.o -MF CMakeFiles/Crane.dir/src/Optimizer.cpp.o.d -o CMakeFiles/Crane.dir/src/Optimizer.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Optimizer.cpp"

CMakeFiles/Crane.dir/src/Optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/Optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Optimizer.cpp" > CMakeFiles/Crane.dir/src/Optimizer.cpp.i

CMakeFiles/Crane.dir/src/Optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/Optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Optimizer.cpp" -o CMakeFiles/Crane.dir/src/Optimizer.cpp.s

CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o: ../src/OptimizerBFGS.cpp
CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o -MF CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o.d -o CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerBFGS.cpp"

CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerBFGS.cpp" > CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.i

CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerBFGS.cpp" -o CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.s

CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o: ../src/OptimizerGD.cpp
CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o -MF CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o.d -o CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerGD.cpp"

CMakeFiles/Crane.dir/src/OptimizerGD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/OptimizerGD.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerGD.cpp" > CMakeFiles/Crane.dir/src/OptimizerGD.cpp.i

CMakeFiles/Crane.dir/src/OptimizerGD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/OptimizerGD.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerGD.cpp" -o CMakeFiles/Crane.dir/src/OptimizerGD.cpp.s

CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o: ../src/OptimizerNM.cpp
CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o -MF CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o.d -o CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerNM.cpp"

CMakeFiles/Crane.dir/src/OptimizerNM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/OptimizerNM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerNM.cpp" > CMakeFiles/Crane.dir/src/OptimizerNM.cpp.i

CMakeFiles/Crane.dir/src/OptimizerNM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/OptimizerNM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/OptimizerNM.cpp" -o CMakeFiles/Crane.dir/src/OptimizerNM.cpp.s

CMakeFiles/Crane.dir/src/Program.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/Program.cpp.o: ../src/Program.cpp
CMakeFiles/Crane.dir/src/Program.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/Crane.dir/src/Program.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/Program.cpp.o -MF CMakeFiles/Crane.dir/src/Program.cpp.o.d -o CMakeFiles/Crane.dir/src/Program.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Program.cpp"

CMakeFiles/Crane.dir/src/Program.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/Program.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Program.cpp" > CMakeFiles/Crane.dir/src/Program.cpp.i

CMakeFiles/Crane.dir/src/Program.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/Program.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Program.cpp" -o CMakeFiles/Crane.dir/src/Program.cpp.s

CMakeFiles/Crane.dir/src/Shape.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/Shape.cpp.o: ../src/Shape.cpp
CMakeFiles/Crane.dir/src/Shape.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/Crane.dir/src/Shape.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/Shape.cpp.o -MF CMakeFiles/Crane.dir/src/Shape.cpp.o.d -o CMakeFiles/Crane.dir/src/Shape.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Shape.cpp"

CMakeFiles/Crane.dir/src/Shape.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/Shape.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Shape.cpp" > CMakeFiles/Crane.dir/src/Shape.cpp.i

CMakeFiles/Crane.dir/src/Shape.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/Shape.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Shape.cpp" -o CMakeFiles/Crane.dir/src/Shape.cpp.s

CMakeFiles/Crane.dir/src/Texture.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/Texture.cpp.o: ../src/Texture.cpp
CMakeFiles/Crane.dir/src/Texture.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/Crane.dir/src/Texture.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/Texture.cpp.o -MF CMakeFiles/Crane.dir/src/Texture.cpp.o.d -o CMakeFiles/Crane.dir/src/Texture.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Texture.cpp"

CMakeFiles/Crane.dir/src/Texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/Texture.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Texture.cpp" > CMakeFiles/Crane.dir/src/Texture.cpp.i

CMakeFiles/Crane.dir/src/Texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/Texture.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/Texture.cpp" -o CMakeFiles/Crane.dir/src/Texture.cpp.s

CMakeFiles/Crane.dir/src/main.cpp.o: CMakeFiles/Crane.dir/flags.make
CMakeFiles/Crane.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/Crane.dir/src/main.cpp.o: CMakeFiles/Crane.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/Crane.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Crane.dir/src/main.cpp.o -MF CMakeFiles/Crane.dir/src/main.cpp.o.d -o CMakeFiles/Crane.dir/src/main.cpp.o -c "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/main.cpp"

CMakeFiles/Crane.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Crane.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/main.cpp" > CMakeFiles/Crane.dir/src/main.cpp.i

CMakeFiles/Crane.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Crane.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/src/main.cpp" -o CMakeFiles/Crane.dir/src/main.cpp.s

# Object files for target Crane
Crane_OBJECTS = \
"CMakeFiles/Crane.dir/src/GLSL.cpp.o" \
"CMakeFiles/Crane.dir/src/Link.cpp.o" \
"CMakeFiles/Crane.dir/src/MatrixStack.cpp.o" \
"CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o" \
"CMakeFiles/Crane.dir/src/Optimizer.cpp.o" \
"CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o" \
"CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o" \
"CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o" \
"CMakeFiles/Crane.dir/src/Program.cpp.o" \
"CMakeFiles/Crane.dir/src/Shape.cpp.o" \
"CMakeFiles/Crane.dir/src/Texture.cpp.o" \
"CMakeFiles/Crane.dir/src/main.cpp.o"

# External object files for target Crane
Crane_EXTERNAL_OBJECTS =

Crane: CMakeFiles/Crane.dir/src/GLSL.cpp.o
Crane: CMakeFiles/Crane.dir/src/Link.cpp.o
Crane: CMakeFiles/Crane.dir/src/MatrixStack.cpp.o
Crane: CMakeFiles/Crane.dir/src/ObjectiveLink.cpp.o
Crane: CMakeFiles/Crane.dir/src/Optimizer.cpp.o
Crane: CMakeFiles/Crane.dir/src/OptimizerBFGS.cpp.o
Crane: CMakeFiles/Crane.dir/src/OptimizerGD.cpp.o
Crane: CMakeFiles/Crane.dir/src/OptimizerNM.cpp.o
Crane: CMakeFiles/Crane.dir/src/Program.cpp.o
Crane: CMakeFiles/Crane.dir/src/Shape.cpp.o
Crane: CMakeFiles/Crane.dir/src/Texture.cpp.o
Crane: CMakeFiles/Crane.dir/src/main.cpp.o
Crane: CMakeFiles/Crane.dir/build.make
Crane: /mnt/c/Users/rlar0/Documents/Programming\ assignment/GBIN/glfw-3.3.9/debug/src/libglfw3.a
Crane: /mnt/c/Users/rlar0/Documents/Programming\ assignment/GBIN/glew-2.1.0/lib/libGLEW.a
Crane: /usr/lib/x86_64-linux-gnu/librt.a
Crane: /usr/lib/x86_64-linux-gnu/libm.so
Crane: /usr/lib/x86_64-linux-gnu/libX11.so
Crane: CMakeFiles/Crane.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_13) "Linking CXX executable Crane"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Crane.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Crane.dir/build: Crane
.PHONY : CMakeFiles/Crane.dir/build

CMakeFiles/Crane.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Crane.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Crane.dir/clean

CMakeFiles/Crane.dir/depend:
	cd "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane" "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane" "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build" "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build" "/mnt/c/Users/rlar0/Documents/Programming assignment/CSCE 450/AnimProjectCrane/build/CMakeFiles/Crane.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/Crane.dir/depend

