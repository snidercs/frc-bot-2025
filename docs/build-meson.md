# Meson
Meson can be used to build native c++ firwmare and native lua using luabot.  This build is a proof-of concept and move toward maturing [luabot](https://github.com/luabot/luabot) so it is comparible to the other FRC-supported build systems.

## Setup
We actually need gradle to pull in the Phoenix6 libraries for use in meson:
```
./gradlew installFrcUserProgramLinuxx86-64ReleaseExecutable --max-workers=2
```

First is to prepare a new directory to contain the build.  Do that, then change in to it.

```
meson setup build-meson
cd build-meson
```

The simulator needs the cmake version of wpilib.  If it isn't installed already, meson will attempt building it as a subproject. The setup console log will show how everything is configured.

## Built It
Meson uses ninja on the backend which can be used directly.
```
ninja -j4
```

## Unit tests
Meson has built-in unit-test capabilities.
```
ninja test
```

And to run the simulator, use the launcher script
```
sh ../util/simulate.sh
```

## WPILib CMake Build
allwpilib should compile as a subproject, but if it is failing then you should compile wpilib with cmake and install it. It is needed for meson to build the simulator.

```
# Clone the repository.
git clone https://github.com/wpilibsuite/allwpilib
cd allwpilib

# Create a build directory and move in to it.
mkdir -p build-cmake
cd build-cmake

cmake .. -G Ninja -DWITH_JAVA=NO -DWITH_DOCS=NO -DWITH_TESTS=NO
ninja
```

If you're on a slow machine or have less than 16G of ram, tone it back.
```
ninja -j4
```

Then install it
```
sudo ninja install
# ... or if that doesn't work ...
sudo cmake --build . --target install
```
