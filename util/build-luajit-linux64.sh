#!/bin/bash

ljdir="$(pwd)/vendordeps/luajit"
sdkdir="$1"

if [ -z "$1" ]; then
    sdkdir="$(pwd)/vendordeps/sdk"
fi

cd "$ljdir"
set -ex

### Build for host machine
make clean
make amalg HOST_CC="gcc -m64 -std=c99" \
    BUILDMODE="static" \
    XCFLAGS="-DLUAJIT_ENABLE_LUA52COMPAT=1" \
    PREFIX="${sdkdir}/linux64"
make install PREFIX="${sdkdir}/linux64"
make clean
