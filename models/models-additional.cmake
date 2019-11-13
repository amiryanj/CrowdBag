set(CRAAL_MODELS_LIBRARIES "")

#MyMath
find_package(mymath)
set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${MYMATH_LIBRARIES})

#Helbing
find_package(helbing REQUIRED)
if(helbing_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${HELBING_LIBRARIES})
    add_definitions(-DHELBING_MODEL)
endif(helbing_FOUND)

#Helbing-3D
find_package(helbing-3d)
if(helbing-3d_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${HELBING-3D_LIBRARIES})
    add_definitions(-DHELBING3D_MODEL)
endif(helbing-3d_FOUND)

#RVO2
find_package(rvo2)
if(rvo2_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${RVO2_LIBRARIES})
    add_definitions(-DRVO2_MODEL)
endif(rvo2_FOUND)

#PowerLaw
find_package(powerlaw)
if(powerlaw_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${POWERLAW_LIBRARIES})
    add_definitions(-DPOWERLAW_MODEL)
endif(powerlaw_FOUND)

#RVO2-3D
find_package(rvo2-3d)
if(rvo2-3d_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${RVO2-3D_LIBRARIES})
    add_definitions(-DRVO2_3D_MODEL)
endif(rvo2-3d_FOUND)

#TModel
find_package(tmodel)
if(tmodel_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${TMODEL_LIBRARIES})
    add_definitions(-DTMODEL_MODEL)
endif(tmodel_FOUND)

#UberModel
find_package(ubermodel)
if(ubermodel_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${UBERMODEL_LIBRARIES})
    add_definitions(-DUBER_MODEL)
endif(ubermodel_FOUND)

#WarpDriver
find_package(warp)
if(warp_FOUND)
    set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${WARP_LIBRARIES})
    add_definitions()
endif(warp_FOUND)

#WildMagic5
find_package(wildmagic5 REQUIRED)
set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${WILDMAGIC5_LIBRARIES})
