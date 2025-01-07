#!/bin/bash

export MACOSX_DEPLOYMENT_TARGET=10.12

ljdir="$(pwd)/vendordeps/luajit"
sdkdir="$(pwd)/vendordeps/sdk"

cd "$ljdir"
set -ex

### Build for arm machine
make clean
make amalg CC="clang -arch arm64 -std=c99" \
    BUILDMODE="static" \
    XCFLAGS="-DLUAJIT_ENABLE_LUA52COMPAT=1" \
    PREFIX="${sdkdir}/macos"
make install PREFIX="${sdkdir}/macos"
make clean

### Set arm .a file aside
mv "${sdkdir}/macos/lib/libluajit-5.1.a" "${sdkdir}/macos/lib/libluajit-5.1-arm.a"

### Build for x86_64 machine

make clean
make amalg CC="clang -arch x86_64 -std=c99" \
    BUILDMODE="static" \
    XCFLAGS="-DLUAJIT_ENABLE_LUA52COMPAT=1" \
    PREFIX="${sdkdir}/macos"
make install PREFIX="${sdkdir}/macos"
make clean

### Set x86_64 .a file aside
mv "${sdkdir}/macos/lib/libluajit-5.1.a" "${sdkdir}/macos/lib/libluajit-5.1-x86_64.a"

### Use lipo to combine the two builds
lipo -create -output "${sdkdir}/macos/lib/libluajit-5.1.a" "${sdkdir}/macos/lib/libluajit-5.1-arm.a" "${sdkdir}/macos/lib/libluajit-5.1-x86_64.a"

