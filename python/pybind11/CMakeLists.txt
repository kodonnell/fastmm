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

# Install to Python site-packages
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_INSTALL_PREFIX ${PYTHON_SITE_PACKAGES} CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
# install(TARGETS fmm LIBRARY DESTINATION "Lib/site-packages")
install(TARGETS fmm LIBRARY DESTINATION .)
