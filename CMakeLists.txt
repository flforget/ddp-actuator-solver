CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(private_cmake/FindqpOASES.cmake)

# Stop if qpOASES is not here
IF (NOT qpOASES_FOUND)
  MESSAGE(ERROR " qpOASES not found. Please install it (using robotpkg for instance)")
ENDIF()

# Compiles despite warnings
SET(CXX_DISABLE_WERROR True)
# Display all warnings
SET(CMAKE_VERBOSE_MAKEFILE True)

# Set project identity
SET(PROJECT_NAME ddp-actuator-solver)
SET(PROJECT_DESCRIPTION "DDP/iLQR solver for robotics actuators command")
SET(PROJECT_URL "https://github.com/stack-of-tasks/ddp-actuator-solver")

SETUP_PROJECT()

# Add eigen3 as another needed dependency
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${qpOASES_INCLUDEDIR})

# Set the headers to be installed
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

# Generate dependency to ddp-actuator-solver-examples in pc file
PKG_CONFIG_APPEND_LIBS(ddp-actuator-solver-examples)
PKG_CONFIG_APPEND_LIBS(${qpOASES_LIBRARIES})

SETUP_PROJECT_FINALIZE()
