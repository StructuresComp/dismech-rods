## DisMech: A Discrete Differential Geometry-based Physical Simulator

<p align="center">
<img src="media/spider_incline.gif" width="600"alt>
<br>
<em> Spider robot created using DisMech's API dropped onto an incline. </em>
</p>

<p align="center">
<img src="media/real2sim.gif" width="600"alt>
<br>
<em> Real2sim soft manipulator example. </em>
</p>

DisMech is a discrete differential geometry-based physical simulator for elastic rod-like structures and soft robots.
Based on the [Discrete Elastic Rods](https://www.cs.columbia.edu/cg/pdfs/143-rods.pdf) framework, it can be used to simulate soft structures for a wide variety of purposes such as robotic deformable material manipulation and soft robot control. 

***

### TODO
If you'd like DisMech to support a new feature, feel free create an issue and we'll add it to the list here.
- [ ] Add detailed documentation for all examples. 
- [ ] Add self-contact.
- [ ] Add shell functionality.
***

### Dependencies

There are some dependencies required prior to compilation.
Instructions for macOS and Ubuntu are similar (presented below).
For other operating systems you should be able to modify the commands below appropriately.

- **macOS**: Because this uses the MKL, it's not certain to run on Apple silicone.
- **macOS**: If you're running a mac, it's highly recommended you use a package manager like [MacPorts](https://www.macports.org/install.php) or [homebrew](https://brew.sh/). Instructions below are for MacPorts.
- **Note**: Some of these packages are installed to the system library for convenience. You may want to install locally to e.g., `~/.local` to avoid conflicts with system libraries. Add the `cmake` flag: `-D CMAKE_INSTALL_PREFIX=~/.local`. Then `sudo` is not required to install. You'll need to ensure subsequent builds know where to find the build libraries.

- X11
  - An X11 (xorg) server is necessary to use the `freeglut` library. This exists already on Linux.
  - **macOS**: This can be installed with MacPorts: `sudo port install xorg-server`. Then log out and back in.
- [Eigen 3.4.0](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - Eigen is used for various linear algebra operations.
  - **macOS**: You can install this version with MacPorts: `sudo port install eigen3`. Otherwise, build instructions are below.
  - DisMech is built with Eigen version 3.4.0 which can be downloaded [here](https://gitlab.com/libeigen/eigen/-/releases/3.4.0). After downloading the source code, install through cmake as follows.
    ```bash
    cd eigen-3.4.0 && mkdir build && cd build
    cmake ..
    sudo make install
    ```
- [SymEngine](https://github.com/symengine/symengine)
  - SymEngine is used for symbolic differentiation and function generation.
  - **macOS**: SymEngine with LLVM can be installed with MacPorts: `sudo port install symengine`.
  - Before installing SymEngine, LLVM is required which can be installed most easily via a package manager:
    - **Ubuntu**: `sudo apt-get install llvm`
    - **macOS**: `sudo port install llvm-15`
  - Afterwards, install SymEngine from source using the following commands.
    ```bash
    git clone https://github.com/symengine/symengine
    cd symengine && mkdir build && cd build
    cmake -D WITH_LLVM=on -D BUILD_BENCHMARKS=off -D BUILD_TESTS=off ..
    make -j4
    sudo make install
    ```
  - **macOS**: You'll need to provide the LLVM root to the build with `-D CMAKE_PREFIX_PATH=/opt/local/libexec/llvm-15` (if installed via MacPorts).
- [Intel oneAPI Math Kernel Library (oneMKL)](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=webdownload&options=online)
  - Necessary for access to Pardiso, which is used as a sparse matrix solver.
  - Intel MKL is also used as the BLAS / LAPACK backend for Eigen.
  - **macOS**: Download from [Intel](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html) and use the install script.
  - **Ubuntu**: Follow the below steps.
    ```bash
    cd /tmp
    wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18483/l_onemkl_p_2022.0.2.136.sh

    # This runs an installer, simply follow the instructions.
    sudo sh ./l_onemkl_p_2022.0.2.136.sh
    ```
  - Add the following to your .bashrc. Change the directory accordingly if your MKL version is different.
    ```bash
    export MKLROOT=/opt/intel/oneapi/mkl/2022.0.2
    ```

- [OpenGL / GLUT](https://www.opengl.org/)
  - OpenGL / GLUT is used for rendering the knot through a simple graphic.
  - Simply install through apt package manager:
    - **Ubuntu**: `sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev`
    - **macOS**: `sudo port install freeglut pkgconfig` (Note: `pkgconfig` is necessary to avoid finding system GLUT instead of `freeglut`.)

- Lapack (*included in MKL*)

***

### Running Examples
DisMech is setup so that simulation environments can be instantiated using a single cpp file called `robotDescription.cpp` and a parameter file in the form of a text file.

Several example of working DisMech simulations can be seen in the `examples/` directory.
In order to run an example, copy the example cpp file into the main directory and then compile DisMech.
For example, using the cantilever beam example:

```bash
cp examples/cantilever_case/cantileverExample.cpp robotDescription.cpp
mkdir build && cd build
cmake ..
make -j4
cd ..
```
Each example has its own corresponding parameter text file. Therefore, in order to run each example simulation, run the `dismech.sh` script along with the respective example parameter file as shown.
```bash
./dismech.sh examples/cantilever_case/cantilever_params.txt
```
If you want to run another example, simply replace the `robotDescription.cpp` file and remake and rerun DisMech with the appropriate parameter file.

***

### Creating Custom Simulation Environments
In case you want to create a custom simulation environment, take a look at the provided examples on how to do so.

Defining the soft structure(s) / robot(s), boundary conditions, custom external forces, and logging are done in `robotDescription.cpp`.

All other parameters are set through a text file fed as an argument to the `dismech.sh` script.

Specifiable parameters are as follows (we use SI units):
- ```rodRadius``` - Cross-sectional radius of the rod.
- ```density``` - Mass per unit volume.
- ```youngM``` - Young's modulus.
- ```Poisson``` - Poisson ratio.
- ```tol``` and ```stol``` - Small numbers used in solving the linear system. Fraction of a percent, e.g. 1.0e-4, is often a good choice.
- ```maxIter``` - Maximum number of iterations allowed before the solver quits.
- ```gVector``` - 3x1 vector specifying acceleration due to gravity.
- ```viscosity``` - Viscosity for applying environmental damping forces.
- ```render (0 or 1)```- Flag indicating whether OpenGL visualization should be rendered.
- ```renderScale```- Scale factor for the rendering. Ignored if `render` is set to 0.
- ```showMatFrames (0 or 1)```- Flag for rendering the material frames. Ignored if `render` is set to 0.
- ```deltaTime``` - Time step size.
- ```delta``` - Distance tolerance for contact.
- ```mu``` - Friction coefficient. A value of zero turns friction off.
- ```nu``` - Slipping tolerance for friction.
- ```lineSearch (0 or 1)``` - Flag indicating whether line search will be used.
- ```simTime``` - Total time for simulation.
- ```enable2DSim (0 or 1)``` - Flag indicating whether 2D simulation is adequate.
- ```enableLogging (0 or 1)``` - Flag indicating whether logging should be enabled.
- ```logfileBase``` - File path where log files should be saved.
- ```loggingPeriod``` - Number of time steps to wait before logging each entry.
- ```debugVerbosity (0 or 1)``` - Dictates whether certain debug messages will be printed.
- ```cmdLinePer``` - Number of time steps to wait before printing sim info to command line. Set to 0 to disable printing.
- ```floorZ``` - Z-coordinate of the floor. Set to -9999 to disable.
- ```adaptiveTimeStepping``` - Turns on adaptive time stepping which halves the time step size is failure to converge after set number of iterations. Only valid for implicit integration schemes. Set to 0 to disable.
- ```integrationScheme``` - Available options are `verlet_position, backward_euler, implicit_midpoint`.
- ```phiCtrlFilePath``` - File path to kappa bar actuation file. Comment out to disable soft limb actuation.
