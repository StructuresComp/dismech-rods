#!/usr/bin/env bash
cp custom_bot/$1.cpp robotDescription.cpp
cd build
make -j4
cd ..

./dismech.sh $2