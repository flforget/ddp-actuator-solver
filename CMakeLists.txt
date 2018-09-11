CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(private_cmake/FindqpOASES.cmake)

IF (NOT qpOASES_FOUND)
  MESSAGE(ERROR " qpOASES not found. Please install it (using robotpkg for instance)")
ENDIF()

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

SET(PROJECT_NAME ddp-actuator-solver)
SET(PROJECT_DESCRIPTION "DDP/iLQR solver for robotics actuators command")
SET(PROJECT_URL "https://github.com/stack-of-tasks/ddp-actuator-solver")

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${qpOASES_INCLUDEDIR})

SET(${PROJECT_NAME}_HEADERS
  include/ddp-actuator-solver/costfunction.hh
  include/ddp-actuator-solver/dynamicmodel.hh
  include/ddp-actuator-solver/ddpsolver.hh)

SET(HEADERS
  ${${PROJECT_NAME}_HEADERS}
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)
ADD_SUBDIRECTORY(examples)

SETUP_PROJECT_FINALIZE()
