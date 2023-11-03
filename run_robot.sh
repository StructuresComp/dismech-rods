#!/usr/bin/env bash
cp custom_bot/customBot.cpp robotDescription.cpp
cd build
make -j4
cd ..

./dismech.sh