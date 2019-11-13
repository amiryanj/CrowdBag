cmake_minimum_required(VERSION 2.6)

#add_subdirectory(mymath)
add_subdirectory(powerlaw)
add_subdirectory(helbing)
#add_subdirectory(helbing-3d)
add_subdirectory(rvo2)
#add_subdirectory(rvo2-3d)
#add_subdirectory(uber)
#add_subdirectory(vision)


#install wildmagic5
if (UNIX)
    execute_process(COMMAND unzip wildmagic5.zip -d ./wildmagic5 WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set(wildmagic5_DIR ${CMAKE_SOURCE_DIR}/wildmagic5/lib/wildmagic5)
    find_package(wildmagic5)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/wildmagic5/include DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/wildmagic5/lib/wildmagic5 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif(UNIX)

if(wildmagic5_FOUND)
    add_subdirectory(tmodel)   
endif(wildmagic5_FOUND)


