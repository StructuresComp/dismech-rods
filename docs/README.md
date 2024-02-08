## Documentation using Doxygen+Breathe+Sphinx

This folder contains all docs related files. This in addition to the .readthedocs.yaml is used to generate the documentation locally and over readthedocs.io.

### Installing dependencies

#### Doxygen
Sphinx doesn’t have the ability to extract API documentation from C++ headers; this needs to be supplied using doxygen. Download and install it from the [official site](https://www.doxygen.nl/download.html). There are binaries for Windows, Linux (Ubuntu), and MacOS, alongside source which you can build yourself.

#### Sphinx
Sphinx can be installed by following the simple instructions from the [official site](https://www.sphinx-doc.org/en/master/usage/installation.html). Sphinx is written in Python, so you need to have python installed. You can simply install Sphinx through pip using 
  ```bash
  pip install -U sphinx
  ```

#### Read the Docs Sphinx Theme
This can be installed through pip using
  ```bash
  pip install sphinx_rtd_theme
  ```
#### Breathe
Breathe is the bridge between Doxygen and Sphinx; taking the output from the former and making it available through some special directives in the latter. It can be installed with pip:
  ```bash
  pip install breathe
  ```

### Instructions on how to create documentation for new/existing code

- Set the `CREATE_DOC_FLAG` to `"true"` in the CMakeLists.txt line 131 in the main directory (it is set to `"false"` by default).
- Build the code similar to how you would build it to run a simulation.
  ```bash
  cp examples/cantilever_case/cantileverExample.cpp robotDescription.cpp
  mkdir build && cd build
  cmake ..
  make -j4
  ```
- You will see the html files created in build/docs/sphinx directory. Open index.html locally to ensure if all the changes are in place.
- Commit all the changes in the docs directory to the repo, as a pull request (readthedocs will build the website itself to check for any issues) or directly (go to readthedocs and click "Build Version").
- Do not commit the documentation files that get created in the build folder.

### Adding comments to the code for effective documentation

- The commenting format for documenting code using Doxygen can be found here: [Documenting the code: Special comment blocks](https://www.doxygen.nl/manual/docblocks.html)
- If you are using vscode, you can simply add an extension for doxygen "Doxygen Documentation Generator". Once you install it, now wherever you want to add comments just type `/**` and press the Enter key. It will autogenerate a comment block which you can populate according to your needs.


### References for developing this documentation pipeline
- [Clear, Functional C++ Documentation with Sphinx + Breathe + Doxygen + CMake](https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/)
- [breathe-apidoc](https://manpages.ubuntu.com/manpages/jammy/man1/breathe-apidoc.1.html)
