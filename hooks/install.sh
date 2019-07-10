#!/bin/bash
set -euo pipefail

# $PWD = hooks/
chmod +x pre-commit
chmod +x pre-commit-clang-format.sh
cd ../.git/hooks
rm pre-commit.sample
ln -s ../../hooks/pre-commit
ln -s ../../hooks/pre-commit-clang-format.sh
cd ../..
# if Windows didn't build clang-format as expected:
# mkdir tools/build/bin
# ln -s ../Debug/bin/clang-format.exe tools/build/bin/clang-format
cd hooks
