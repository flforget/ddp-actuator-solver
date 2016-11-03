SET(${PROJECT_NAME}_HEADERS
        costfunctionromeoactuator.h
        romeolinearactuator.h
        config.h
        dynamicmodel.h
        romeosimpleactuator.h
        romeotorqueactuator.h
        ceaactuator.h
        costfunction.h
        ilqrsolver.h
        types.h
)

INSTALL(FILES ${${PROJECT_NAME}_HEADER} DESTINATION include)

