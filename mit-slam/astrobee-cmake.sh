#!/usr/bin/env bash

# Write make files for the currently active astrobee branch.

source ~/repos/astrobee/build/devel/setup.bash
cd ~/repos/astrobee/freeflyer-shared-td/
./scripts/configure.sh -l -F -D -p ../freeflyer-install/ -b ../freeflyer-build/
