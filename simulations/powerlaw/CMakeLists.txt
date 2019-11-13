cmake_minimum_required(VERSION 2.6)
		
#Project
project(PowerLaw)

#Includes
include_directories(include)
include_directories(include/proximitydatabase)
include_directories(include/util)

#Headers
set(POWERLAW_HEADERS
        include/PowerLawAgent.h
	include/AgentInitialParameters.h
	include/AnticipatoryModel.h
	include/LineObstacle.h
	include/SimulationEngine.h
	include/proximitydatabase/lq2D.h
	include/proximitydatabase/Proximity2D.h
	include/proximitydatabase/ProximityDatabaseItem.h
	include/util/Definitions.h
	include/util/Vector2D.h
)

#Source Files
file(
	GLOB
        POWERLAW_SOURCES
        src/PowerLawAgent.cpp
	src/LineObstacle.cpp
	src/lq2D.cpp
	src/SimulationEngine.cpp
)

#Output
add_library(
        powerlaw SHARED
        ${POWERLAW_SOURCES}
)

#Install
install(FILES include/Agent.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw)
install(FILES include/AgentInitialParameters.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw)
install(FILES include/AnticipatoryModel.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw)
install(FILES include/LineObstacle.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw)
install(FILES include/SimulationEngine.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw)
install(FILES include/proximitydatabase/lq2D.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw/proximitydatabase)
install(FILES include/proximitydatabase/Proximity2D.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw/proximitydatabase)
install(FILES include/proximitydatabase/ProximityDatabaseItem.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw/proximitydatabase)
install(FILES include/util/Definitions.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw/util)
install(FILES include/util/Vector2D.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/powerlaw/util)

install(TARGETS powerlaw DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/powerlaw EXPORT powerlaw-targets)
install(EXPORT powerlaw-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/powerlaw)
install(FILES powerlaw-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/powerlaw)

