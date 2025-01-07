[![Build](https://github.com/snidercs/frc-bot-2025/actions/workflows/build.yml/badge.svg)](https://github.com/snidercs/frc-bot-2025/actions/workflows/build.yml)
# Robot Firmware 2025
## Features
- Built-in Lua interpreter.
- Smart bot configuration with Lua bindings and negligible overhead.
- Fast code deploys when c++ doesn't need recompiled.
- Ultra low latency performance.
- Design is decoupled from wpilib making it portable to other robotics systems.

## Clone
```bash
git clone --recursive https://github.com/snidercs/frc-bot-2025.git
```

## Clang Format
The c++ code should be formatted before submitting pull requests. Do this with python or gradle.

```bash
# Run the python script directly
python3 util/format.py
# or do it wrapped in a gradle task
./gradlew clangFormat
```

## Requirements

### Dependencies
The `gradle build` and `gradle deploy` tasks both need roboRIO libraries and headers in place.  Most of them are handled by wpilib, but some need special attention.

Note: For _Windows_ GitBash and the CMD prompt are both needed.

#### LuaJIT
The build requires LuaJIT, which WPIlib does not provide.  It must be compiled manually.

**Linux**
```bash
# multilib support is needed for cross building, install if needed
sudo apt-get install gcc-multilib g++-multilib
# Run the native/roborio build scripts
util/build-luajit-linux64.sh
util/build-luajit-roborio.sh
```

**macOS**

Make sure you have an amd64 version of container
```bash
docker build . -t snidercs/bot-2024 --platform=linux/amd64
```

The Mac build script can produce `arm64` or `x86_64` binaries.  It will select the system default if not specified.
```bash
util/build-luajit-macos.sh # Builds for both arm64 and x86_64

# Docker is required for roboRio builds/deploys
util/docker-run.sh util/build-luajit-roborio.sh
```

**Windows**

The build script can be run from a **GitBash** terminal:
```bash
# In a bash emulator
util/build-luajit-msvc.sh
```

The roboRIO binaries need docker to compile from Windows which _requires_ a regular **CMD prompt**:
```
util\docker-run.bat util/build-luajit-roborio.sh
```

### Firmware Build with WPIlib VSCode
After LuaJIT is compiled, open a terminal and do:
```bash
./gradlew build
```
This will make roboRio binaries to be deployed
### Firmware With Docker
```bash
./dockerbuild.sh
```

## Testing
Run all unit tests.
```bash
./gradlew check
```

## Deployment
Run the following command to deploy code to the roboRIO
```bash
./gradlew deploy
```

If it gives problems, cleaning the project could help. The `--info` option could give more information too.
```bash
./gradlew clean
./gradlew deploy --info
```
