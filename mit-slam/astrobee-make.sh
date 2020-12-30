#!/usr/bin/env bash

# Build the current makefiles for astrobee.

source ~/repos/astrobee/build/devel/setup.bash
cd ~/repos/astrobee/freeflyer-build/
make -j6
