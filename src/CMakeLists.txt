# Copyright 2020, 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set(CMAKE_CXX_STANDARD 14) 
# Install python module
if(NOT DEFINED PYTHON_SITELIB)
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" "-c"
    #"import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'dist-packages']))"
    "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix='', plat_specific=False))"
    OUTPUT_VARIABLE PYTHON_SITELIB
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()

install(FILES
  agimus_demos/__init__.py
  agimus_demos/calibration.py
  agimus_demos/in_state_planner.py
  agimus_demos/tools_hpp.py
  DESTINATION ${PYTHON_SITELIB}/agimus_demos
  )

install(FILES
  agimus_demos/talos/tools_hpp.py
  agimus_demos/talos/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_demos/talos
  )

IF(USE_TIAGO)
  find_package(pybind11 CONFIG)
  if(pybind11_FOUND)
    pybind11_add_module(pytsp tsp.cpp tsp-py.cpp)
    target_link_libraries(pytsp PRIVATE Eigen3::Eigen)

    message(STATUS "Installing python module to ${PYTHON_SITELIB}")
    install(TARGETS pytsp
      DESTINATION ${PYTHON_SITELIB}/agimus_demos)
  endif()
ENDIF()

install(FILES
  rqt_agimus/tooltip_calibration.py
  rqt_agimus/contact_calibration.py
  DESTINATION ${PYTHON_SITELIB}/rqt_agimus)

SET(LIBRARY_NAME ${PROJECT_NAME})
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  contact-simulation.cc
  contact-simulation.hh
  )
target_link_libraries (${LIBRARY_NAME} PUBLIC
    dynamic-graph-python::dynamic-graph-python
    )
INSTALL(TARGETS ${LIBRARY_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )

add_library(wrap MODULE python-module-py.cc)
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/agimus_demos/sot/${SUBMODULENAME})
set_target_properties(wrap
  PROPERTIES PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/src
  OUTPUT_NAME agimus_demos/sot/wrap)
target_compile_options(wrap PRIVATE "-Wl,--no-as-needed")
target_link_libraries(wrap PUBLIC
  dynamic-graph-python::dynamic-graph-python
  ${PYTHON_LIBRARY} ${LIBRARY_NAME})
target_include_directories(wrap PRIVATE ${PYTHON_INCLUDE_PATH})
install(TARGETS wrap
  DESTINATION ${PYTHON_SITELIB}/agimus_demos/sot)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/agimus_demos/sot/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_demos/sot)
