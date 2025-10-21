# pybind11 integration for FASTMM Python bindings

cmake_minimum_required(VERSION 3.15)

# Use modern Python discovery to match the interpreter cibuildwheel selected
# This avoids mismatches like including Python 3.8 headers but linking against 3.13 libs
set(PYBIND11_FINDPYTHON ON)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

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

# Install Python extension (.pyd) - MODULE libraries use LIBRARY destination even on Windows
install(TARGETS fastmm
    LIBRARY DESTINATION .
    COMPONENT python)

# Install FASTMMLIB.dll - RUNTIME for shared libraries on Windows
install(TARGETS FASTMMLIB
    RUNTIME DESTINATION .
    COMPONENT python)

# Install Python source files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/fastmm/
    DESTINATION .
    COMPONENT python
    FILES_MATCHING PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.egg-info" EXCLUDE)
