cmake_minimum_required(VERSION 3.11)
project(dismech-rods)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")

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

link_directories(${MKL_LIBRARY_DIR})

add_executable(simDER
        src/main.cpp
        src/world.cpp
        src/robotDescription.cpp

        src/initialization/setInput.cpp

        src/rod_mechanics/baseForce.cpp
        src/rod_mechanics/elasticRod.cpp
        src/rod_mechanics/elasticStretchingForce.cpp
        src/rod_mechanics/elasticBendingForce.cpp
        src/rod_mechanics/elasticTwistingForce.cpp
        src/rod_mechanics/externalGravityForce.cpp
        src/rod_mechanics/dampingForce.cpp
        src/rod_mechanics/inertialForce.cpp
        src/rod_mechanics/symbolicEquations.cpp
        src/rod_mechanics/floorContactForce.cpp
        src/rod_mechanics/elasticJoint.cpp
        # src/rod_mechanics/collisionDetector.cpp
        # src/rod_mechanics/contactPotentialIMC.cpp

        src/time_steppers/baseTimeStepper.cpp
        src/time_steppers/explicitTimeStepper.cpp
        src/time_steppers/verletPosition.cpp
        src/time_steppers/implicitTimeStepper.cpp
        src/time_steppers/backwardEuler.cpp
        src/time_steppers/implicitMidpoint.cpp

        src/solvers/baseSolver.cpp
        src/solvers/pardisoSolver.cpp
        src/solvers/dgbsvSolver.cpp

        src/logging/worldLogger.cpp

        src/simulation_environments/derSimulationEnvironment.cpp
        src/simulation_environments/headlessDERSimulationEnvironment.cpp
        src/simulation_environments/openglDERSimulationEnvironment.cpp

        src/controllers/rodController.cpp
        src/controllers/rodOpenLoopFileKappabarSetter.cpp
        )

target_link_libraries(simDER PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        MKL::MKL
        Eigen3::Eigen
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}

        lapack

        # gfortran
        stdc++fs
        Threads::Threads
        m
        ${SYMENGINE_LIBRARIES})

if (OpenMP_CXX_FOUND)
    target_link_libraries(simDER PUBLIC OpenMP::OpenMP_CXX)
endif ()
