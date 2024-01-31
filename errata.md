- Install [libccd](https://github.com/danfis/libccd) with the following commands, making sure to build shared libraries:
     ```bash
    git clone https://github.com/danfis/libccd
    cd libccd/src
    cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
    make -j4
    sudo make install
     ```
- There is a slight difference: ```cd libccd/src``` and not ```cd libccd```
