cmake_minimum_required(VERSION 3.11)
project(dismech-rods)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-O3 -DNDEBUG -Wno-deprecated-declarations")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if (COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
set(MAGNUM_WITH_SDL2APPLICATION ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Per https://eigen.tuxfamily.org/dox/TopicUsingIntelMKL.html eigen should be used
# with MKL LAPACK only with lp64 set. The default may be ILP64, which we don't want.
set(MKL_INTERFACE_FULL intel_lp64)

add_definitions(-DEIGEN_USE_MKL_ALL)

find_package(SymEngine REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(MKL CONFIG REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(OpenMP)
find_package(FCL REQUIRED)

# Run with -DWITH_MAGNUM=ON to build with Magnum library.
option(WITH_MAGNUM OFF)
if (WITH_MAGNUM)
    add_definitions(-DWITH_MAGNUM)
    find_package(Corrade REQUIRED Main PluginManager Utility)
    find_package(Magnum REQUIRED
            GL
            MeshTools
            Primitives
            Shaders
            Trade
            Sdl2Application
            SceneGraph)
    find_package(MagnumPlugins REQUIRED PngImageConverter)
else ()
    message(STATUS "Building with Magnum disabled")
endif ()

option(WITH_PYBIND OFF)
if (WITH_PYBIND)
    find_package(pybind11 REQUIRED)
else ()
    message(STATUS "Building with pybind disabled")
endif ()


if (${HAVE_SYMENGINE_LLVM})
    message(STATUS "Testing SymEngine LLVM & SBML support - found")
else ()
    message(WARNING "SymEngine library is missing LLVM and/or SBML support")
    message(WARNING "${SYMENGINE_LLVM_ERROR_LOG}")
endif ()

include_directories(${SYMENGINE_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        MKL::MKL)

set(SOURCES
        src/world.cpp

        src/rod_mechanics/baseForce.cpp
        src/rod_mechanics/elasticRod.cpp
        src/rod_mechanics/elasticJoint.cpp
        src/rod_mechanics/softRobots.cpp

        src/rod_mechanics/forceContainer.cpp

        src/rod_mechanics/inner_forces/inertialForce.cpp
        src/rod_mechanics/inner_forces/elasticStretchingForce.cpp
        src/rod_mechanics/inner_forces/elasticBendingForce.cpp
        src/rod_mechanics/inner_forces/elasticTwistingForce.cpp

        src/rod_mechanics/external_forces/gravityForce.cpp
        src/rod_mechanics/external_forces/dampingForce.cpp
        src/rod_mechanics/external_forces/floorContactForce.cpp
        src/rod_mechanics/external_forces/symbolicEquations.cpp
        src/rod_mechanics/external_forces/uniformConstantForce.cpp
        src/rod_mechanics/external_forces/collisionDetector.cpp
        src/rod_mechanics/external_forces/contactForce.cpp

        src/time_steppers/baseTimeStepper.cpp

        src/time_steppers/explicitTimeStepper.cpp
        src/time_steppers/forwardEuler.cpp
        src/time_steppers/verletPosition.cpp

        src/time_steppers/implicitTimeStepper.cpp
        src/time_steppers/backwardEuler.cpp
        src/time_steppers/implicitMidpoint.cpp

        src/solvers/baseSolver.cpp
        src/solvers/pardisoSolver.cpp
        src/solvers/dgbsvSolver.cpp

        src/logging/worldLogger.cpp
        src/logging/rodNodeLogger.cpp
        src/logging/velocityLogger.cpp

        src/simulation_environments/derSimulationEnvironment.cpp
        src/simulation_environments/headlessDERSimulationEnvironment.cpp
        src/simulation_environments/openglDERSimulationEnvironment.cpp

        src/controllers/baseController.cpp
        src/controllers/openLoopUniformKappaBarController.cpp
        src/controllers/activeEntanglementController.cpp

        src/utils/utils.cpp
)

if (WITH_MAGNUM)
    set(SOURCES ${SOURCES}
            src/simulation_environments/magnumDERSimulationEnvironment.cpp
    )
endif ()

set(LINK_LIBRARIES
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        MKL::MKL
        Eigen3::Eigen
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}
        lapack
        stdc++fs
        Threads::Threads
        m
        ${SYMENGINE_LIBRARIES}
        fcl
)

if (WITH_MAGNUM)
    set(LINK_LIBRARIES ${LINK_LIBRARIES}
            Corrade::Main
            Corrade::PluginManager
            Corrade::Utility
            Magnum::Application
            Magnum::GL
            Magnum::Magnum
            Magnum::MeshTools
            Magnum::Primitives
            Magnum::SceneGraph
            Magnum::Shaders
            MagnumPlugins::PngImageConverter
    )
endif ()

if (OpenMP_CXX_FOUND)
    set (LINK_LIBRARIES ${LINK_LIBRARIES}
            OpenMP::OpenMP_CXX
    )
endif ()

link_directories(${MKL_LIBRARY_DIR})

add_library(common_sources STATIC ${SOURCES})

target_include_directories(common_sources PUBLIC
        src
)

target_link_libraries(common_sources PUBLIC
        ${LINK_LIBRARIES}
)


# Run with cmake --fresh -DCREATE_DOCS=ON .. only when documentation needs to be updated.

# If docs change and you want to preview the updates in index.html, you'll need to rm -rf * and
# remake everything in build. Don't make clean && make. Kind of annoying but this should only be
# done for previewing. All updates to the actual docs will be done more fluidly.

# If wanting to rebuild source code, will have to cmake --fresh .. again.

# Probably a good idea to split the documentation cmake process into its own directory later.
#option(CREATE_DOCS OFF)
#
#if (CREATE_DOCS)
#    add_subdirectory("docs")
#endif ()

if (EXISTS ${CMAKE_SOURCE_DIR}/robotDescription.cpp)
    add_executable(disMech
            src/main.cpp
            robotDescription.cpp
    )

    target_link_libraries(disMech PRIVATE
            common_sources
    )
else()
    message(WARNING "No robotDescription.cpp found. No executable will be created.")
endif()

if (WITH_PYBIND)
    pybind11_add_module(py_dismech
            src/app.cpp
    )
    target_link_libraries(py_dismech PRIVATE
            common_sources
    )

    set_target_properties(py_dismech PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/py_dismech"
)
endif()


option(CREATE_EXAMPLES OFF)

set(EXAMPLES
        examples/active_entanglement_case/activeEntanglementExample.cpp
        examples/cantilever_case/cantileverExample.cpp
        examples/friction_case/frictionExample.cpp
        examples/helix_case/helixExample.cpp
        examples/horton_case/hortonExample.cpp
        examples/real2sim_case/real2simExample.cpp
        examples/spider_case/spiderExample.cpp
)
if (CREATE_EXAMPLES)
    # Loop through each example file and create a build target
    foreach(EXAMPLE ${EXAMPLES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
        get_filename_component(EXAMPLE_DIR ${EXAMPLE} DIRECTORY)

        add_executable(${EXAMPLE_NAME}
                src/main.cpp
                ${EXAMPLE}
        )

        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/${EXAMPLE_DIR}
        )

        target_link_libraries(${EXAMPLE_NAME} PRIVATE
            common_sources
        )
    endforeach()
endif ()
