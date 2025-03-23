![Build](https://github.com/snidercs/frc-bot-2025/actions/workflows/build.yml/badge.svg)
# Robot Firmware FRC 2025
## Features
- Primary firmware in Python
- Experimental firmwares with LuaJIT FFI and C++
- Build automation and testing
- Phoenix6 powered drivetrain

## Get the Code
Git is required for development.  Cloning should be possible inside vscode, or use the command line:

```bash
git clone https://github.com/snidercs/frc-bot-2025.git
# ... or if you have write access ...
git clone git@github.com:snidercs/frc-bot-2025.git
```

## Code Style & Best Practices
Please follow the code style as best as you can.  See [docs/codestyle.md](docs/codestyle.md) for details.

## Building

### Requirements
*All platforms* need to have the [latest WPIlib](https://github.com/wpilibsuite/allwpilib/releases) installed as well as [Python](https://www.python.org/downloads/). If you're wanting to dabble in Lua and build-systems development, then you'll also need [Meson](https://mesonbuild.com/Getting-meson.html) and [CMake](https://cmake.org).

## Linux
Development using Linux is *highly recommended* as all the tools are readily available.  The following command will install the above (minus wpilib) plus some extras needed for compiling WPIlib directly.

```bash
sudo apt install build-essential gcc-multilib g++-multilib cmake python3 python3-pip meson protobuf-compiler libprotobuf-dev libprotoc-dev libopencv-dev clang
```

## macOS
* **python**: Robotpy recommends the official installer.
* **Commandline Tools**: Provided by Xcode
* **Homebrew**: For anything not alredy provided

## Windows
* **python**: Robotpy recommends the official installer.
* **Visual Studio**: For the C++ compiler.


### LuaJIT ffi bindings
The primary firmware is in python to start with, while the others are needed for the development of LuaJIT ffi bindings.

* [Gradle](docs/build-gradle.md) (C++)
* [Meson](docs/build-meson.md) (C++ / LuaJIT)
* [Docker](docs/docker.md) (CI / misc)

### Robotpy
* **[Robotpy](docs/build-python.md)** (python)

```bash
cd frc-bot-2025/pybot
robotpy sync # This will Download RoboRIO requirements and installs requirements locally
robotpy deploy --skip-tests # To avoid all of the dumb questions
# or
robotpy sim # To run the simulator
```
  

