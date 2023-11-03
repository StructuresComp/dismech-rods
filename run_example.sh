#!/usr/bin/env bash
cp examples/$1_case/$1Example.cpp robotDescription.cpp
cd build
make -j4
cd ..

./dismech.sh $2