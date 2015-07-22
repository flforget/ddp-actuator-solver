include_directories(include)

ADD_LIBRARY(DDP ${SRC})

install(TARGETS DDP DESTINATION lib)

SET(PROJECT_LIBS DDP)

add_executable(execmain main.cpp)

TARGET_LINK_LIBRARIES(execmain ${PROJECT_LIBS})
install(TARGETS execmain DESTINATION bin)


