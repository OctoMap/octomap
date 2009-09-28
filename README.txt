INSTALLATION:
##################

Build the complete project by changing into the "build" directory 
and running cmake:
	
	cmake ../src
	
Type "make" to compile afterwards. This will create all CMake
files cleanly in the "build" folder (Out-of-source build).
Executables will end up in "bin", libraries in "lib".


A debug configuration can be created by running:
	
	cmake -DCMAKE_BUILD_TYPE=Debug ../src

in "build" or a different directory (e.g. "build-debug").


Eclipse project files can be generated (with some limitations, see:
http://www.vtk.org/Wiki/Eclipse_CDT4_Generator) by running:

	cmake -G"Eclipse CDT4 - Unix Makefiles" ../src 
	
Import the project (existing project, root is the build folder, do
not copy contents) into Eclipse afterwards.



