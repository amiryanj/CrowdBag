cmake_minimum_required(VERSION 3.5)
project (crowd_bag)
set (CMAKE_CXX_STANDARD 11)
set (PYTHON3 FALSE CACHE BOOL "Using python3 instead of python2")
set (WITH_ROS FALSE CACHE BOOL "a ROS interface")
if (UNIX AND NOT APPLE)
    message("Python Version Suffix:  ${PYTHON_VERSION_SUFFIX}")
    # if (PYTHON_VERSION_MAJOR EQUAL 3)
    if (PYTHON3)
        message("===== Compile with Python 3 =====")
        FIND_PACKAGE(Boost COMPONENTS python-py35)
        FIND_PACKAGE(PythonInterp 3)
        FIND_PACKAGE(PythonLibs 3 REQUIRED)
    else()
        message("===== Compile with Python 2 =====")
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
        models/src/SimulationHelbing.cpp
        models/src/SimulationPowerLaw.cpp
        models/src/SimulationRVO2.cpp
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

file (  GLOB
        THIRD_PARTY_SOURCES
        ./third-party/tinyxml2.cpp
)

add_subdirectory(simulations/powerlaw)
add_subdirectory(simulations/helbing)
add_subdirectory(simulations/rvo2)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR} ./craal-core/include ./models/include ./third-party)
INCLUDE_DIRECTORIES(./simulations/helbing/include
                    ./simulations/rvo2/include
                    ./simulations/powerlaw/include)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES}) # Deprecated but so convenient!

PYTHON_ADD_MODULE(crowdbag crowdbag_py.cpp crowdbag.hpp
    ${CORE_SOURCES} ${MODELS_SOURCES}
    )
target_link_libraries(crowdbag helbing rvo2 powerlaw)

add_executable(testcpp test.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(testcpp helbing rvo2 powerlaw)



add_executable(1to1_parallel scenario_1to1_parallel.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(1to1_parallel helbing rvo2 powerlaw)

add_executable(1to1_crossing scenario_1to1_crossing.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(1to1_crossing helbing rvo2 powerlaw)

add_executable(1to1_45crossing scenario_1to1_45crossing.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(1to1_45crossing helbing rvo2 powerlaw)



add_executable(1to10_parallel scenario_1to10_parallel.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES}  ${THIRD_PARTY_SOURCES})
target_link_libraries(1to10_parallel helbing rvo2 powerlaw)

add_executable(1to10_crossing scenario_1to10_crossing.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES} )
target_link_libraries(1to10_crossing helbing rvo2 powerlaw)

add_executable(1to10_45crossing scenario_1to10_45crossing.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES} )
target_link_libraries(1to10_45crossing helbing rvo2 powerlaw)



add_executable(circle5 scenario_circle5.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(circle5 helbing rvo2 powerlaw)

add_executable(circle10 scenario_circle10.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(circle10 helbing rvo2 powerlaw)

add_executable(circle15 scenario_circle15.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES})
target_link_libraries(circle15 helbing rvo2 powerlaw)


add_executable(oneway scenario_oneway.cpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES} )
target_link_libraries(oneway helbing rvo2 powerlaw)


if (WITH_ROS) # when user wants to install ros
    ## Find catkin and any catkin packages
    message("===== Compile with ROS =====")
#    find_package(catkin REQUIRED )
    find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

    include_directories(${catkin_INCLUDE_DIRS})
    link_directories(${catkin_LIBRARY_DIRS})

    ## Declare ROS messages and services
#    add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
#    add_message_files(DIRECTORY msg FILES Num.msg)

    ## Generate added messages and services
#    generate_messages(DEPENDENCIES std_msgs)

    ## Declare a catkin package
#    catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

    add_executable(crowdbag_node crowdbag_node.cpp
        crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES}
        )
    target_link_libraries(crowdbag_node
        ${catkin_LIBRARIES} helbing rvo2 powerlaw)
    #add_dependencies()
endif()

FILE(COPY test.py DESTINATION .)
#ADD_TEST(NAME 02-ExposingClasses COMMAND ${PYTHON_EXECUTABLE} classes.py)
