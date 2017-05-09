#!/usr/bin/env bash
cd cmake-build-release
make -j12
make doc
make package
cp cobotsys-0.0.1-Linux.tar.gz ../../tutorial/

cd ../../tutorial
rm -rf cobotsys-0.0.1-Linux
tar -xf cobotsys-0.0.1-Linux.tar.gz

cd ../cobotsys