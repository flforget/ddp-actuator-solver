SET(${PROJECT_NAME}_HEADERS
        costfunctionromeoactuator.h
        romeolinearactuator.h
        config.h
        dynamicmodel.h
        romeosimpleactuator.h
        romeoactuatorpos.h
        costfunction.h
        ilqrsolver.h
        costfunctionpneumaticarmelbow.h
        pneumaticarm_2linkmodel.hh
)

INSTALL(FILES ${${PROJECT_NAME}_HEADER} DESTINATION include)

