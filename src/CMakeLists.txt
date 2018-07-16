
SET(source_files
  costfunctionromeoactuator.cpp
  costtemp.cpp
  dctemp.cpp
  romeosimpleactuator.cpp
  romeotorqueactuator.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${source_files})

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen3)
