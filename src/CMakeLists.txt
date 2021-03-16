find_package(pybind11 CONFIG)
if(pybind11_FOUND)
  pybind11_add_module(pytsp tsp.cpp tsp-py.cpp)
  target_link_libraries(pytsp PRIVATE Eigen3::Eigen)

  if(NOT DEFINED PYTHON_SITELIB)
    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" "-c"
      #"import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'dist-packages']))"
      "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix='', plat_specific=False))"
      OUTPUT_VARIABLE PYTHON_SITELIB
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
  endif()

  message(STATUS "Installing python module to ${PYTHON_SITELIB}")
  install(TARGETS pytsp
    DESTINATION ${PYTHON_SITELIB}/agimus_demos)
  install(FILES __init__.py
    DESTINATION ${PYTHON_SITELIB}/agimus_demos)
endif()
