cmake_minimum_required(VERSION 3.10)

project(acadogenquad)

# Set the paths to your ACADO installation
set(ACADO_DIR /path/to/acado)

# Include ACADO headers
include_directories(/usr/local/include/acado)

# Link ACADO libraries
link_directories(${ACADO_DIR}/lib)

# Add the executable
add_executable(acadogenquad src/acadogenquad.cpp)

# Link the ACADO library
target_link_libraries(acadogenquad /usr/local/lib/libacado_toolkit_s.so)
