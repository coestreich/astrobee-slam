# scripts

Trajectory generation using ACADO is taken care of here. The trajectory optimization definition is contained in `NMPC.m`. This MATLAB file also autocodes to C++, which is then pulled into the nominal Astrobee build process.

## Usage

Steps to export ACADO C++ code and libraries for Astrobee use:

- Write code for NMPC solver in `NMPC.m` (the solver consists of only the first part, the rest is code for simulation test, take it with grain of salt). ACADO will generate a folder called `NMPC_export` containing C code for the solver. Make sure your ACADO installation is in your MATLAB path!

- Place the `NMPC_export` folder under `nmpc_astrobee/`, if it is not already there

- The exported C code is now ready for compilation to produce a static library. This is already taken care of in CMakeLists.txt---just use the normal Astrobee build process.