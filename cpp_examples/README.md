### Running Examples in C++ (Deprecated)

Note that running DisMech simulations directly in C++ is deprecated. 
To use DisMech, it's suggested to use the Python bindings instead.

***

DisMech is setup so that simulation environments can be instantiated using a single cpp file
called `robot_description.cpp`.

Several example of working DisMech simulations can be seen in the `examples/` directory.
In order to run an example, copy the example cpp file into the main directory and then compile DisMech.
For example, using the cantilever beam example:

```bash
cp examples/cantilever_case/cantilever_example.cpp robot_description.cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
cd ..
```
Afterwards, simply run the simulation using the `dismech.sh` script.
```bash
./dismech.sh
```
Modifications to the examples can be made easily by changing the parameters shown below.
If you want to run another example, simply replace the `robot_description.cpp` file and recompile.
Users can also simply build all examples from the start. To do so, run the following:
```bash
mkdir build && cd build
cmake -DCREATE_EXAMPLES=on ..
make -j$(nproc)
cd ..
# Make sure to run from the main directory as some examples use relative paths
OMP_NUM_THREADS=1 ./examples/spider_case/spider_example_exec
```
The Pardiso solver can be parallelized by setting the env variable `OMP_NUM_THREADS > 1`. For all the systems defined
in `/examples`, their Jacobian matrices are small enough that any amount of parallelization actually slows down the
simulation. Therefore, it is recommended to set `OMP_NUM_THREADS=1` (`dismech.sh` does this as automatically) and see if parallelization is worth it
for larger systems through profiling.

### Creating Custom Simulation Environments
In case you want to create a custom simulation environment, take a look at the provided examples on how to do so.

Model and environment parameters such as defining the soft structure(s) / robot(s), boundary
conditions, forces, and logging are done solely in `robot_description.cpp` to avoid large recompile times.
