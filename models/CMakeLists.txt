cmake_minimum_required(VERSION 2.8.8)

# Project
project(Models)

#include(${CMAKE_CURRENT_LIST_DIR}/models-additional.cmake)

# Includes
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../craal-core/include)

# Headers
set(MODELS_HEADERS
#	${CMAKE_CURRENT_LIST_DIR}/include/pedestrianVision.h
	${CMAKE_CURRENT_LIST_DIR}/include/Sampler.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationBoids.h
        ${CMAKE_CURRENT_LIST_DIR}/include/SimulationBoids3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationRandomBoids3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationHelbing.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationHelbing3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationPowerLaw.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationRVO2.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationRVO23D.h
#	${CMAKE_CURRENT_LIST_DIR}/include/SimulationTModel.h
#        ${CMAKE_CURRENT_LIST_DIR}/include/SimulationUber.h
#        ${CMAKE_CURRENT_LIST_DIR}/include/SimulationWarp.h
	PARENT_SCOPE
)

# Source Files
file(	GLOB
        MODELS_SOURCES
#        ${CMAKE_CURRENT_LIST_DIR}/src/pedestrianVision.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Sampler.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationBoids.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationBoids3D.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationRandomBoids3D.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationHelbing.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationHelbing3D.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationPowerLaw.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationRVO2.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationRVO23D.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationTModel.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationUber.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/src/SimulationWarp.cpp
)

# Output
add_library(
        models STATIC
        ${MODELS_SOURCES}
)
