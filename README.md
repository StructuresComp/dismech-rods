## [Implicit Contact Model (IMC)](https://asmedigitalcollection.asme.org/appliedmechanics/article/88/5/051010/1099667/Implicit-Contact-Model-for-Discrete-Elastic-Rods)

Contact model for 3D elastic rod simulations. Uses [Discrete Elastic Rod (DER)](http://www.cs.columbia.edu/cg/pdfs/143-rods.pdf) framework and incorporates contact and friction. Formulates a contact potential as a twice differentiable analytical expression through smooth approximations 
and uses the subsequent energy gradient (forces) and Hessian (force Jacobian) to simulate contact and friction. Simulation examples using IMC to resolve contact and friction can be seen below in Figure 1.


<p align="center">
<img src="images/knot_tying.png" alt>
<br>
<em> Figure 1. Simulation examples for tying overhand knots with various unknotting numbers. </em>
</p>

***

### Formulation Updates since the Published Paper
- Explicit and hybrid formulations for IMC were removed. After a Hessian chain ruling bug fix, the fully implicit version is by far superior in terms of performance.
- Friction has been changed to a fully implicit formulation.
- Smooth distance has been exchanged for piecewise analytical distance.
- For full updates, please refer to our new paper [here](https://arxiv.org/abs/2205.10309).

***

## How to Use

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

All simulation parameters are set through a parameter file ```option.txt```. A template file ```template_option.txt``` is provided that can be used to construct ```option.txt```.

```bash
cp template_option.txt option.txt   # create option.txt
```
Specifiable parameters are as follows (we use SI units):
- ```RodLength``` - Contour length of the rod.
- ```numVertices``` - Number of nodes on the rod.
- ```rodRadius``` - Cross-sectional radius of the rod.
- ```helixradius``` - Radius of the helix.
- ```helixpitch``` - Pitch of the helix.
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
- ```waitTime``` - Initial wait period duration.
- ```pullTime``` - Duration to pull for (*starts after ```waitTime``` is done*).
- ```releaseTime``` - Duration to loosen for (*starts after ```waitTime``` + ```pullTime``` is done*).
- ```pullSpeed``` - Speed at which to pull and/or loosen each end.
- ```deltaTime``` - Time step size.
- ```colLimit``` - Distance limit for inclusion in contact candidate set (*colLimit must be > delta*).
- ```delta``` - Distance tolerance for contact.
- ```kScaler``` - Constant scaling factor for contact stiffness.
- ```mu``` - Friction coefficient. A value of zero turns friction off.
- ```nu``` - Slipping tolerance for friction.
- ```lineSearch (0 or 1)``` - Flag indicating whether line search will be used.
- ```knotConfig``` - File name for the initial knot configuration. Should be a txt file located in ```knot_configurations``` directory. Note that overhand knot configurations for ```n1, n2, n3, n4``` are provided with a discretization of 301 nodes.

***
### Running the Simulation
Once parameters are set to your liking, the simulation can be ran from the terminal by running the provided script:
```bash
./run.sh
```
If this doesn't work, execute ```chmod +x run.sh``` prior to running.

***

### Citation
If our work has helped your research, please cite the following paper.
```
@article{choi_imc_2021,
    author = {Choi, Andrew and Tong, Dezhong and Jawed, Mohammad K. and Joo, Jungseock},
    title = "{Implicit Contact Model for Discrete Elastic Rods in Knot Tying}",
    journal = {Journal of Applied Mechanics},
    volume = {88},
    number = {5},
    year = {2021},
    month = {03},
}

@misc{tong_imc_2022,
  author = {Tong, Dezhong and Choi, Andrew and Joo, Jungseock and Jawed, M. Khalid},
  title = {A Fully Implicit Method for Robust Frictional Contact Handling in Elastic Rods},
  publisher = {arXiv},
  year = {2022},
  doi = {10.48550/ARXIV.2205.10309},
  url = {https://arxiv.org/abs/2205.10309},
}

```



