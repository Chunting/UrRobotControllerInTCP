#!/bin/bash


rtk_libode_path=$1

cd ode-0.13
pwd
./configure --enable-double-precision --enable-shared --prefix $rtk_libode_path
