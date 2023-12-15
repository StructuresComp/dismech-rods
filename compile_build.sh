#!/usr/bin/env bash
if [ -d "build" ]; then
  echo "Deleting previous build..."
  rm -rf build
fi

echo "Creating new build..."
cp custom_bot/customBot.cpp robotDescription.cpp
mkdir build 
cd build
cmake ..
make -j4
cd ..

echo "Build complete."