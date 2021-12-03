cmake_minimum_required(VERSION 3.5)
project (crowdbag)
set (CMAKE_CXX_STANDARD 11)
set (PYTHON3 FALSE CACHE BOOL "Using python3 instead of python2")
set (WITH_ROS FALSE)

if(CATKIN_DEVEL_PREFIX)
    set (WITH_ROS TRUE)
    message("[CrowdBag] Using Catkin !!!!")
endif()

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
        models/src/Sampler.cpp
        models/src/SimulationBoids.cpp
        models/src/SimulationHelbing.cpp
        models/src/SimulationPowerLaw.cpp
        models/src/SimulationRVO2.cpp
#        models/src/SimulationTModel.cpp
)

file(	GLOB
        CORE_SOURCES
        ./craal-core/src/Simulation.cpp
        ./craal-core/src/Configurable.cpp
        ./craal-core/src/PDF.cpp
#        ./craal-core/src/ResourceLister.cpp
#        ./craal-core/src/Calibration.cpp
#        ./craal-core/src/ConfigurationParser.cpp
#        ./craal-core/src/Pedestrian.cpp
#        ./craal-core/src/Dummy.cpp
#        ./craal-core/src/Experiment.cpp
#        ./craal-core/src/Metric.cpp
#        ./craal-core/src/Record.cpp
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

add_executable(testcpp test.cpp xml_loader.hpp crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES})
target_link_libraries(testcpp helbing rvo2 powerlaw)

if (WITH_ROS) # when user wants to install ros
    ## Find catkin and any catkin packages
    message("[CrowdBag] Using Catkin !!!!")
    message("===== Compile with ROS =====")
    find_package(catkin REQUIRED COMPONENTS roscpp rospy genmsg std_msgs geometry_msgs)

    include_directories(${catkin_INCLUDE_DIRS})
    link_directories(${catkin_LIBRARY_DIRS})

    ## Declare ROS messages and services
    add_message_files(DIRECTORY msg FILES SimPedestrianState.msg CrowdState.msg)
    add_service_files(DIRECTORY srv FILES SimulateCrowd.srv)

    ## Generate added messages and services
    generate_messages(DEPENDENCIES std_msgs geometry_msgs)

    ## Declare a catkin package
    catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
    # catkin_package()

    add_executable(crowdbag_node crowdbag_node.cpp
        crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES}
        )
    target_link_libraries(crowdbag_node
        ${catkin_LIBRARIES} helbing rvo2 powerlaw)

    add_executable(crowdbag_service crowdbag_server.cpp
        crowdbag.hpp ${CORE_SOURCES} ${MODELS_SOURCES} ${THIRD_PARTY_SOURCES})
    target_link_libraries(crowdbag_service
        ${catkin_LIBRARIES} helbing rvo2 powerlaw)


    #add_dependencies()
endif()

FILE(COPY test.py DESTINATION .)
#ADD_TEST(NAME 02-ExposingClasses COMMAND ${PYTHON_EXECUTABLE} classes.py)
