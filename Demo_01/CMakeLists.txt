cmake_minimum_required(VERSION 3.0.0)

# set the project name and a version number
project(demo VERSION 1.0)

# this modeule determines the opencv and threads library of the system
find_package( OpenCV REQUIRED)
find_package( Threads REQUIRED )

# add library
# normal libraries, object libraries, alias libraries, interface libraries
add_library(code
    Demo.cpp
    Demo.h
    Helper.cpp
    Helper.h
)

# set target(code) properties
set_target_properties(code PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# specify libraries and flags to use when linking target code
# PUBLIC: libraries and targets are linked and made part of the link interface
# PRIVATE: libraries and targets are linked to, but are not part of the link interface
target_link_libraries(code
    PUBLIC
        ${OpenCV_LIBS}
)

# add an excecutable target called: main 
# main has to be built from the source files listed in the command invocation
add_executable(main
    main.cpp
)

# set target "main" properties
set_target_properties(main PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

#
target_link_libraries(main
    PRIVATE
        code
)


# add executable for a target called:unit_test
add_executable(unit_test
     unit_test.cpp
)

# set target "unit_test" properties
set_target_properties( unit_test PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_link_libraries(unit_test
    PRIVATE
        code
)

# to distribute project to other people so they can use it
# build installation package that support binary installations and package management features
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
