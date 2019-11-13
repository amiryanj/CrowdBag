cmake_minimum_required(VERSION 3.5)
project (crowd_sim)
set (CMAKE_CXX_STANDARD 11)
set (PYTHON3 FALSE CACHE BOOL "Set when using python 3")

if (UNIX AND NOT APPLE)
    message("Here:  ${PYTHON_VERSION_SUFFIX}")
    # if (PYTHON_VERSION_MAJOR EQUAL 3)
    if (PYTHON3)
        message("== Compile with Python 3 ==")
        FIND_PACKAGE(Boost COMPONENTS python-py35)
        FIND_PACKAGE(PythonInterp 3)
        FIND_PACKAGE(PythonLibs 3 REQUIRED)
    else()
        message("== Compile with Python 2 ==")
        FIND_PACKAGE(Boost COMPONENTS python)
        FIND_PACKAGE(PythonInterp)
        FIND_PACKAGE(PythonLibs REQUIRED)
    endif()
else()
    if (PYTHON_VERSION_MAJOR EQUAL 3)
        FIND_PACKAGE(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
        FIND_PACKAGE(PythonInterp 3)
        FIND_PACKAGE(PythonLibs 3 REQUIRED)
    else()
        FIND_PACKAGE(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
        FIND_PACKAGE(PythonInterp)
        FIND_PACKAGE(PythonLibs REQUIRED)
    endif()
endif()

#add_subdirectory(./craal-core)
#add_subdirectory(./models)

file(	GLOB
        MODELS_SOURCES
#        models/src/pedestrianVision.cpp
        models/src/Sampler.cpp
        models/src/SimulationBoids.cpp
#        models/src/SimulationBoids3D.cpp
#        models/src/SimulationRandomBoids3D.cpp
        models/src/SimulationHelbing.cpp
#        models/src/SimulationHelbing3D.cpp
        models/src/SimulationPowerLaw.cpp
        models/src/SimulationRVO2.cpp
#        models/src/SimulationRVO23D.cpp
#        models/src/SimulationTModel.cpp
#        models/src/SimulationUber.cpp
#        models/src/SimulationWarp.cpp
)

file(	GLOB
        CORE_SOURCES
        ./craal-core/src/Calibration.cpp
        ./craal-core/src/Configurable.cpp
        ./craal-core/src/ConfigurationParser.cpp
        ./craal-core/src/Dummy.cpp
        ./craal-core/src/Experiment.cpp
        ./craal-core/src/Experiment3D.cpp
        ./craal-core/src/Metric.cpp
        ./craal-core/src/PDF.cpp
        ./craal-core/src/Pedestrian.cpp
        ./craal-core/src/Record.cpp
        ./craal-core/src/Record3D.cpp
        ./craal-core/src/ResourceLister.cpp
        ./craal-core/src/Simulation.cpp
        ./craal-core/src/Simulation3D.cpp
)

add_subdirectory(simulations/powerlaw)
add_subdirectory(simulations/helbing)
add_subdirectory(simulations/rvo2)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR} ./craal-core/include ./models/include)
INCLUDE_DIRECTORIES(./simulations/helbing/include
                    ./simulations/rvo2/include
                    ./simulations/powerlaw/include)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES}) # Deprecated but so convenient!

PYTHON_ADD_MODULE(crowdsim crowd_sim.cpp
    ${CORE_SOURCES} ${MODELS_SOURCES}
    )
target_link_libraries(crowdsim
        helbing rvo2 powerlaw)

#add_executable(test1 main.cpp crowd_sim.cpp ${CORE_SOURCES} ${MODELS_SOURCES})
#target_link_libraries(test1
#        ${CMAKE_CURRENT_BINARY_DIR}/simulations/helbing/libhelbing.so
#        ${CMAKE_CURRENT_BINARY_DIR}/simulations/rvo2/librvo2.so
#        ${CMAKE_CURRENT_BINARY_DIR}/simulations/powerlaw/libpowerlaw.so )

FILE(COPY test.py DESTINATION .)
#ADD_TEST(NAME 02-ExposingClasses COMMAND ${PYTHON_EXECUTABLE} classes.py)
