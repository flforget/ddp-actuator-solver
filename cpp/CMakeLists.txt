CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CXX_DISABLE_WERROR True)

SET(PROJECT_NAME DDPsolver)
# SET(PROJECT_DESCRIPTION "DDP/iLQR solver for robotics actuators command")
# SET(PROJECT_URL "")

SET(LIBRARY_OUTPUT_PATH ${PROJECT_NAME}/lib)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(test)

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

FILE(
    GLOB_RECURSE
    source_files
    include/*
    src/*
)

ADD_EXECUTABLE(main test/main.cpp ${source_files})
ADD_EXECUTABLE(mainMPC test/mainMPC.cpp ${source_files})
# ADD_EXECUTABLE(testModel test/testModel.cpp ${source_files})
