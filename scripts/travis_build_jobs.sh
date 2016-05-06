#!/bin/bash

# travis build script for test compilations

set -e

function build {
  cd $1
  mkdir build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/tmp/octomap/$1
  make -j4
  cd ..
}

case "$1" in
"dist")
  build .
  cd build && make test
  make install
  ;;
"components")
  build octomap
  cd build && make test
  make install
  cd ../..
  build dynamicEDT3D
  cd ..
  build octovis
  cd ..
  ;;
*)
  echo "Invalid build variant"
  exit 1
esac




