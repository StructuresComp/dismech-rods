cmake_minimum_required(VERSION 3.4)
project(TriangleExample)

set(MAGNUM_WITH_SDL2APPLICATION ON)

set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES /usr/local/lib ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})
set(CMAKE_PREFIX_PATH /usr/local ${CMAKE_PREFIX_PATH})
set(CMAKE_LIBRARY_PATH /usr/local/lib ${CMAKE_LIBRARY_PATH})


find_package(Corrade REQUIRED Main PluginManager Utility)
find_package(Magnum REQUIRED
    GL
    MeshTools
    Primitives
    Shaders
    Trade
    Sdl2Application
    SceneGraph)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(magnum-primitives PrimitivesExample.cpp)
target_link_libraries(magnum-primitives PRIVATE
    Corrade::Main
    Corrade::PluginManager
    Corrade::Utility
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::Shaders
    Magnum::Trade)

add_executable(magnum-mouseinteraction MouseInteractionExample.cpp)
target_link_libraries(magnum-mouseinteraction PRIVATE
    Corrade::Main
    Corrade::PluginManager
    Corrade::Utility
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders)
