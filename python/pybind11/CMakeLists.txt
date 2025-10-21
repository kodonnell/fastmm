# pybind11 integration for FMM Python bindings

cmake_minimum_required(VERSION 3.5.1)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

# Add pybind11 (header-only)
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.12.0
)
FetchContent_MakeAvailable(pybind11)

include_directories(${PYTHON_INCLUDE_DIRS})

set(FMM_SRC ${PROJECT_SOURCE_DIR}/python/pybind11/fmm_bindings.cpp)



add_library(fmm MODULE ${FMM_SRC})
set_target_properties(fmm PROPERTIES PREFIX "" OUTPUT_NAME "fmm")
if (WIN32)
  set_target_properties(fmm PROPERTIES SUFFIX ".pyd")
endif()
target_link_libraries(fmm PRIVATE pybind11::pybind11)

# Link to core FMM library and Python
set(FMM_LINK_LIBS FMMLIB ${PYTHON_LIBRARIES})
target_link_libraries(fmm PRIVATE ${FMM_LINK_LIBS})

# Use C++14
set_target_properties(fmm PROPERTIES CXX_STANDARD 14)

# Install Python extension (.pyd) - MODULE libraries use LIBRARY destination even on Windows
install(TARGETS fmm
    LIBRARY DESTINATION .
    COMPONENT python)

# Install FMMLIB.dll - RUNTIME for shared libraries on Windows
install(TARGETS FMMLIB
    RUNTIME DESTINATION .
    COMPONENT python)

# Install Python source files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/fmm/
    DESTINATION .
    COMPONENT python
    FILES_MATCHING PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.egg-info" EXCLUDE)
