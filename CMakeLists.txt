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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
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

option(WITH_PYBIND "Build with Python bindings" ON)
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

set(SOURCES
        src/world.cpp

        src/rod_mechanics/base_force.cpp
        src/rod_mechanics/elastic_rod.cpp
        src/rod_mechanics/elastic_joint.cpp
        src/rod_mechanics/soft_robots.cpp

        src/rod_mechanics/force_container.cpp

        src/rod_mechanics/inner_forces/inertial_force.cpp
        src/rod_mechanics/inner_forces/elastic_stretching_force.cpp
        src/rod_mechanics/inner_forces/elastic_bending_force.cpp
        src/rod_mechanics/inner_forces/elastic_twisting_force.cpp

        src/rod_mechanics/external_forces/gravity_force.cpp
        src/rod_mechanics/external_forces/damping_force.cpp
        src/rod_mechanics/external_forces/floor_contact_force.cpp
        src/rod_mechanics/external_forces/symbolic_equations.cpp
        src/rod_mechanics/external_forces/uniform_constant_force.cpp
        src/rod_mechanics/external_forces/collision_detector.cpp
        src/rod_mechanics/external_forces/contact_force.cpp

        src/time_steppers/base_time_stepper.cpp

        src/time_steppers/explicit_time_stepper.cpp
        src/time_steppers/forward_euler.cpp
        src/time_steppers/verlet_position.cpp

        src/time_steppers/implicit_time_stepper.cpp
        src/time_steppers/backward_euler.cpp
        src/time_steppers/implicit_midpoint.cpp

        src/solvers/base_solver.cpp
        src/solvers/pardiso_solver.cpp
        src/solvers/dgbsv_solver.cpp

        src/logging/base_logger.cpp
        src/logging/position_logger.cpp
        src/logging/velocity_logger.cpp

        src/simulation_environments/base_sim_env.cpp
        src/simulation_environments/headless_sim_env.cpp
        src/simulation_environments/opengl_sim_env.cpp

        src/controllers/base_controller.cpp
        src/controllers/open_loop_uniform_kappa_bar_controller.cpp
        src/controllers/active_entanglement_controller.cpp

        src/utils/utils.cpp
)

if (WITH_MAGNUM)
    set(SOURCES ${SOURCES}
            src/simulation_environments/magnum_sim_env.cpp
    )
endif ()

set(LINK_LIBRARIES
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        MKL::MKL
        Eigen3::Eigen
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

if (WITH_PYBIND)
    set(LINK_LIBRARIES ${LINK_LIBRARIES}
            pybind11::module
    )
endif ()

if (OpenMP_CXX_FOUND)
    set (LINK_LIBRARIES ${LINK_LIBRARIES}
            OpenMP::OpenMP_CXX
    )
endif ()

add_library(common_sources STATIC ${SOURCES})

target_include_directories(common_sources PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_include_directories(common_sources SYSTEM PRIVATE
        Eigen3::Eigen
        ${SYMENGINE_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS})

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

if (EXISTS ${CMAKE_SOURCE_DIR}/robot_description.cpp)
    add_executable(disMech
            src/main.cpp
            robot_description.cpp
    )

    target_link_libraries(disMech PRIVATE
            common_sources
    )
else()
    message(WARNING "No robot_description.cpp found. No executable will be created.")
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
        cpp_examples/active_entanglement_case/active_entanglement_example.cpp
        cpp_examples/cantilever_case/cantilever_example.cpp
        cpp_examples/friction_case/friction_example.cpp
        cpp_examples/helix_case/helix_example.cpp
        cpp_examples/horton_case/horton_example.cpp
        cpp_examples/real2sim_case/real2sim_example.cpp
        cpp_examples/spider_case/spider_example.cpp
)
if (CREATE_EXAMPLES)
    # Loop through each example file and create a build target
    foreach(EXAMPLE ${EXAMPLES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
        get_filename_component(EXAMPLE_DIR ${EXAMPLE} DIRECTORY)

        set(EXEC_NAME "${EXAMPLE_NAME}_exec")

        add_executable(${EXEC_NAME}
                src/main.cpp
                ${EXAMPLE}
        )

        set_target_properties(${EXEC_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/${EXAMPLE_DIR}
        )

        target_link_libraries(${EXEC_NAME} PRIVATE
            common_sources
        )
    endforeach()
endif ()
