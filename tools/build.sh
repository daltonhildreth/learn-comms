#!/bin/bash
# Largely as per the official getting started page of clang with minor
# convenience changes: https://clang.llvm.org/get_started.html
set -euo pipefail

# $PWD = tools
git -c submodule."tools/clang".update=checkout submodule update --init clang
git clone git@github.com:llvm-mirror/llvm.git -b release_80 --depth=1
ln -s ../../clang llvm/tools/clang
mkdir build
cd build
cmake ../llvm -DCMAKE_BUILD_TYPE=Release
cmake --build . --target clang-format --parallel
cd ..
