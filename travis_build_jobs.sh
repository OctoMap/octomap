#!/bin/bash

# travis build script for test compilations

set -e

if [ -z "$1" ]
then
  DIR="."
else
  DIR=$1
fi

cd $DIR
mkdir build
cd build
cmake ..
make -j4

# TODO: add test and install jobs



