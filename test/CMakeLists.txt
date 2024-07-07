cmake_minimum_required(VERSION 3.4)
project(TriangleExample)

set(MAGNUM_WITH_SDL2APPLICATION ON)

find_package(Corrade REQUIRED Main)
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
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders)