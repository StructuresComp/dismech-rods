## Dismech-Rods

### How to Use

### Dependencies
Install the following C++ dependencies:
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - Eigen is used for various linear algebra operations.
  - IMC is built with Eigen version 3.4.0 which can be downloaded [here](https://gitlab.com/libeigen/eigen/-/releases/3.4.0). After downloading the source code, install through cmake as follows.
    ```bash
    cd eigen-3.4.0 && mkdir build && cd build
    cmake ..
    sudo make install
    ```
- [SymEngine](https://github.com/symengine/symengine)
  - SymEngine is used for symbolic differentiation and function generation.
  - Before installing SymEngine, LLVM is required which can be installed through apt.
    ```bash
    sudo apt-get install llvm
    ```
  - Afterwards, install SymEngine from source using the following commands.
    ```bash
    git clone https://github.com/symengine/symengine    
    cd symengine && mkdir build && cd build
    cmake -DWITH_LLVM=on ..
    make -j4
    sudo make install
    ```
- [Intel oneAPI Math Kernel Library (oneMKL)](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=webdownload&options=online)
  - Necessary for access to Pardiso, which is used as a sparse matrix solver.
  - Intel MKL is also used as the BLAS / LAPACK backend for Eigen.
  - If you are using Linux, follow the below steps. Otherwise, click the link above for your OS.
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
      ```bash
    sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
    ```
- Lapack (*usually preinstalled on your computer*)

***
### Compiling
After completing all the necessary above steps, clone the source repository of IMC and then build the project through cmake.
```bash
mkdir build && cd build
cmake ..
make -j4
```

***

### Setting Parameters

To prevent personal robot configurations / simulation parameters from constantly being pushed, there are two template files that are stored on the repository. When starting to use the code, the template files must be copied to create local copies that are ignored by git using the script below.
```bash
./init_local_config.sh
```
All simulation parameters are then set through a parameter file ```option.txt``` while the robot config is designed through the API in ```src/robotDescription.cpp```.

Specifiable parameters are as follows (we use SI units):
- ```rodRadius``` - Cross-sectional radius of the rod.
- ```density``` - Mass per unit volume.
- ```youngM``` - Young's modulus.
- ```Poisson``` - Poisson ratio.
- ```tol``` and ```stol``` - Small numbers used in solving the linear system. Fraction of a percent, e.g. 1.0e-3, is often a good choice.
- ```maxIter``` - Maximum number of iterations allowed before the solver quits. 
- ```gVector``` - 3x1 vector specifying acceleration due to gravity.
- ```viscosity``` - Viscosity for applying damping forces.
- ```render (0 or 1) ```- Flag indicating whether OpenGL visualization should be rendered.
- ```saveData (0 or 1)``` - Flag indicating whether pull forces and rod end positions should be reocrded.
- ```recordNodes (0 or 1)``` - Flag indicating whether nodal positions will be recorded.
- ```dataResolution``` - Rate of data recording in seconds. Applies to both ```saveData``` and ```recordNodes```.
- ```deltaTime``` - Time step size.
- ```colLimit``` - Distance limit for inclusion in contact candidate set (*colLimit must be > delta*).
- ```delta``` - Distance tolerance for contact.
- ```kScaler``` - Constant scaling factor for contact stiffness.
- ```mu``` - Friction coefficient. A value of zero turns friction off.
- ```nu``` - Slipping tolerance for friction.
- ```lineSearch (0 or 1)``` - Flag indicating whether line search will be used.
- ```floorZ``` - Z-coordinate for floor plane.
- ```simTime``` - Total time for simulation.

***
### Running the Simulation
Once parameters are set to your liking, the simulation can be ran from the terminal by running the provided script:
```bash
./run.sh
```
If this doesn't work, execute ```chmod +x run.sh``` prior to running.
