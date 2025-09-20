#!/bin/bash
set -e

echo "----------------"
echo "remove build dir"
echo "----------------"
rm -rf build
echo "----------------"

echo "create build dir"
echo "----------------"
mkdir build && cd build
echo "----------------"

echo "Running cmake"
echo "----------------"
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/.local
cmake --build . --config Release --target install -- -j $(nproc)
echo "-------------------"
