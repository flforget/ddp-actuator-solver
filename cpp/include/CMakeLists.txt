SET(${PROJECT_NAME}_HEADERS
    costfunctionromeoactuator.h
    romeolinearactuator.h
    config.h
    dynamicmodel.h
    romeosimpleactuator.h
    costfunction.h
    ilqrsolver.h
    costfunctionpneumaticarmelbow.h
    pneumaticarmelbowlinear.h
)

INSTALL(FILES ${${PROJECT_NAME}_HEADER} DESTINATION include)

