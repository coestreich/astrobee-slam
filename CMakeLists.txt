cmake_minimum_required(VERSION 3.5)
project(roam-ros VERSION 0.0.0)

# Include the external git submodules, which include PMC and TEASER++ for
# registration.
add_subdirectory(external)

# Include the library to the SLAM pipeline.
add_subdirectory(mit-slam)
