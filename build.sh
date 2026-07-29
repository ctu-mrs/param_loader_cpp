#!/bin/bash
set -e

cp examples/config/test.yaml /tmp/test.yaml
echo "----------------"
echo "remove build dir"
echo "----------------"
rm -rf build
echo "----------------"

echo "create build dir"
echo "----------------"
mkdir build && cd build
echo "----------------"

echo "running cmake"
echo "----------------"
cmake ..
cmake --build . --config Release
echo "-------------------"
