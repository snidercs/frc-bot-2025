#!/bin/sh

ljdir="$(pwd)/vendordeps/luajit"
sdkdir="$1"

if [ -z "$1" ]; then
    sdkdir="$(pwd)/vendordeps/sdk"
fi

cd "$ljdir"
set -ex

### Build for roboRIO
export PATH="$HOME/wpilib/2024/roborio/bin:$PATH"

make clean
make amalg HOST_CC="gcc -m32 -std=c99" \
    CROSS=arm-frc2024-linux-gnueabi- \
    XCFLAGS="-DLUAJIT_ENABLE_LUA52COMPAT=1" \
    BUILDMODE="static" \
    PREFIX="$sdkdir/roborio"
make install PREFIX="$sdkdir/roborio"

make clean
