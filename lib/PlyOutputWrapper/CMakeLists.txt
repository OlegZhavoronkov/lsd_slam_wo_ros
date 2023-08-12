cmake_minimum_required(VERSION 3.18)
#set(LIBRARY_NAME "tmk_logging")
project(plyoutwrapper VERSION 0.1.0 LANGUAGES C CXX) 
set(LIBRARY_NAME ${PROJECT_NAME})

#include(CTest)
#enable_testing()
#
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

#
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)


add_library(${LIBRARY_NAME} SHARED "")
set_target_properties(${LIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)


set(PRIVATE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MeshOutputWrapper.cpp
    )

set(PUBLIC_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PlyOutputWrapper/MeshOutputWrapper.h
)

target_sources(${PROJECT_NAME}
    PRIVATE ${PRIVATE_SRCS}
    INTERFACE ${PUBLIC_HDRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include/
                                           PRIVATE  ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/tinyply/source
                                                    )
target_link_libraries(${PROJECT_NAME}
                    PRIVATE tinyply
                            ${G2O_LIBRARIES}
                            g3logger
                            activeobject 
                            videoio
                    PUBLIC fmt
                    )

