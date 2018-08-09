
macro(_local_test_ddp)
  set(list_var "${ARGN}")
  foreach(loop_var IN LISTS list_var)
    ADD_EXECUTABLE(${loop_var} ${loop_var}.cpp)
    TARGET_LINK_LIBRARIES(${loop_var} ${PROJECT_NAME})
    PKG_CONFIG_USE_DEPENDENCY(${loop_var} qpOASES)
    PKG_CONFIG_USE_DEPENDENCY(${loop_var} eigen3)
  endforeach()
endmacro()


_local_test_ddp(main mainMPC testModel)
#ADD_EXECUTABLE(benchControl benchControl.cpp ${PROJECT_NAME})



