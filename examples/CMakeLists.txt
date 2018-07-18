
macro(_local_test_ddp)
  set(list_var "${ARGN}")
  foreach(loop_var IN LISTS list_var)
    ADD_EXECUTABLE(${loop_var} ${loop_var}.cpp)
    TARGET_LINK_LIBRARIES(${loop_var} ${PROJECT_NAME})
    PKG_CONFIG_USE_DEPENDENCY(${loop_var} qpOASES)
    PKG_CONFIG_USE_DEPENDENCY(${loop_var} eigen3)
  endforeach()
endmacro()

SET(source_files
  romeo_actuator/costfunctionromeoactuator.cpp
  romeo_actuator/romeosimpleactuator.cpp
  romeo_actuator/romeotorqueactuator.cpp
  temperature_control/costtemp.cpp
  temperature_control/dctemp.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${source_files})

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen3)

ADD_SUBDIRECTORY(romeo_actuator)
ADD_SUBDIRECTORY(temperature_control)
