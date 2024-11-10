FROM ubuntu:jammy

# Install all available dependencies via apt
RUN apt update && apt install -y libeigen3-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev llvm wget build-essential cmake pybind11-dev ffmpeg git libfcl-dev libgmp-dev libblas-dev liblapack-dev zlib1g-dev libpng-dev libsdl2-dev

# Install intel MKL
RUN wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
RUN echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list
RUN apt update && apt install -y intel-oneapi-mkl-devel

# symengine
RUN git clone https://github.com/symengine/symengine && cd symengine && mkdir build && cd build && cmake -D WITH_LLVM=on -D BUILD_BENCHMARKS=off -D BUILD_TESTS=off .. && make -j4 && make install

# VIEWER: Magnum and dependencies
RUN git clone https://github.com/mosra/corrade.git && cd corrade && mkdir build && cd build && cmake .. && make -j4 && make install
RUN git clone https://github.com/mosra/magnum.git && cd magnum &&  mkdir build && cd build && cmake -DMAGNUM_WITH_SDL2APPLICATION=ON .. && make -j4 && make install
RUN git clone https://github.com/mosra/magnum-plugins && cd magnum-plugins && mkdir build && cd build && cmake -DMAGNUM_WITH_PNGIMAGECONVERTER=ON .. && make -j4 && make install

# dismech-rods
ENV MKL_DIR=/opt/intel/oneapi/mkl/2024.2
RUN apt install -y libpython3-dev python3-pip
RUN git clone https://github.com/StructuresComp/dismech-rods.git && cd dismech-rods && mkdir build && cd build && cmake -DWITH_MAGNUM=ON -DWITH_PYBIND=ON -DCREATE_EXAMPLES=on .. && make -j4 && cd .. && pip install -e .
RUN apt install -y python3-numpy
