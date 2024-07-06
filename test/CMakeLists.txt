cmake_minimum_required(VERSION 3.4)
project(TriangleExample)

set(MAGNUM_WITH_SDL2APPLICATION ON)

#find_package(Magnum REQUIRED
#    GL
#    Shaders
#    Sdl2Application)
#find_package(SDL2 REQUIRED)
#
#add_executable(test test.cpp)
#target_link_libraries(test PRIVATE
#    Magnum::Magnum
#    Magnum::GL
#    Magnum::Shaders
#        Magnum::Sdl2Application
#    SDL2::SDL2
#    SDL2::SDL2main)
#
#message(STATUS "Magnum include directories: ${MAGNUM_INCLUDE_DIR}")
#message(STATUS "Magnum include directories: ${MAGNUM_INCLUDE_DIRS}")
#message(STATUS "SDL2 include directories: ${SDL2_INCLUDE_DIRS}")
#
#
## Add Magnum include directories
#target_include_directories(test PRIVATE /usr/include/Magnum ${SDL2_INCLUDE_DIRS})
#
find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED
    GL
    MeshTools
    Primitives
    Shaders
    Trade
    Sdl2Application)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(magnum-primitives WIN32 PrimitivesExample.cpp)
target_link_libraries(magnum-primitives PRIVATE
    Corrade::Main
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::Shaders
    Magnum::Trade)