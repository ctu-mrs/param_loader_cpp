#!/bin/bash
set -e

echo "Create build dir"
echo "----------------"
meson setup builddir
echo "----------------"

echo "Running meson"
echo "----------------"
meson compile -C builddir
meson install -C builddir
echo "-------------------"
