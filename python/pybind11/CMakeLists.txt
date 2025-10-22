# pybind11 integration for FASTMM Python bindings

cmake_minimum_required(VERSION 3.15)

# Use modern Python discovery to match the interpreter cibuildwheel selected
# This avoids mismatches like including Python 3.8 headers but linking against 3.13 libs
# Use Development.Module instead of Development - only needs headers for extension modules
set(PYBIND11_FINDPYTHON ON)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

# Add pybind11 (header-only)
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.12.0
)
FetchContent_MakeAvailable(pybind11)

# Include dirs provided by the Python3::Module / pybind11 targets

set(FASTMM_SRC ${PROJECT_SOURCE_DIR}/python/pybind11/fastmm_bindings.cpp)

# Build the Python extension with pybind11 helper to ensure correct Python linkage
pybind11_add_module(fastmm MODULE ${FASTMM_SRC})
set_target_properties(fastmm PROPERTIES PREFIX "" OUTPUT_NAME "fastmm")
if (WIN32)
  set_target_properties(fastmm PROPERTIES SUFFIX ".pyd")
endif()
target_link_libraries(fastmm PRIVATE pybind11::pybind11 Python3::Module)

# Link to core FASTMM library and Python
set(FASTMM_LINK_LIBS FASTMMLIB)
target_link_libraries(fastmm PRIVATE ${FASTMM_LINK_LIBS})

# Use C++14
set_target_properties(fastmm PROPERTIES CXX_STANDARD 14)

# Set RPATH so the extension can find libfastmmlib.so in the same directory
if(UNIX AND NOT APPLE)
    set_target_properties(fastmm PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN"
    )
elseif(APPLE)
    set_target_properties(fastmm PROPERTIES
        BUILD_RPATH "@loader_path"
        INSTALL_RPATH "@loader_path"
    )
endif()

# Install Python extension (.pyd) - MODULE libraries use LIBRARY destination even on Windows
message(STATUS "Configuring install for fastmm.pyd")
install(TARGETS fastmm
    LIBRARY DESTINATION .
    COMPONENT python)

# Install FASTMMLIB shared library
# RUNTIME for Windows (.dll), LIBRARY for Linux/macOS (.so/.dylib)
message(STATUS "Configuring install for FASTMMLIB: $<TARGET_FILE:FASTMMLIB>")
message(STATUS "FASTMMLIB will install RUNTIME to . and LIBRARY to . with component python")
install(TARGETS FASTMMLIB
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
    ARCHIVE DESTINATION .
    COMPONENT python)

# Install Python source files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/fastmm/
    DESTINATION .
    COMPONENT python
    FILES_MATCHING PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.egg-info" EXCLUDE)
