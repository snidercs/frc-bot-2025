# Code Style
Guidlines for hand-written code.

## Casing

Generally speaking: we use camel case in OOP and snake when it makes sense to match a dominant library.  Not strict rules really as long as the casing is consistent.  The exception is wpilibc.  It uses pascal case **which is forbidden** our hand-written code.

## Clang Format
The c++ code should be formatted before submitting pull requests. Depending on your build system of choice...

```bash
# Run the python script directly
python3 util/format.py
# or run the gradle task
./gradlew clangFormat
# or use ninja (from inside the build-meson directory)
ninja clang-format
```

## Filenaming
1. Use `.hpp` for C++, use `.h` for plain C.
1. all lower case
1. no spaces
1. no dashes
1. no underscores

**Good**
```
someclass.hpp
someclass.cpp
utils.hpp
```

**Bad**
```
some-other-class.hpp
some_other_class.cpp

```

## Headers

* Always start with `#pragma once` for the header guard.
* Avoid including headers in other headers when possible; e.g. us c++ forward declares (this is key to faster compile times)

### Private code

* Place private helpers in the `detail` namespace and make sure they are `static`

**Good**
```c++
namespace detail {

static bool checkSomthingSpecial() { 
    // do stuff
    return true;
}

} // namespace detail

void SomeClass::doSomething() {
    if (! detail::checkSomethingSpecial())
        throw std::runtime_error();
    // continue on...
}
```

**Bad**
```c++

// non-static makes it available everywhere with `extern`
// no namespace opens up to symbol conflicts and ambiguity.
bool checkSomthingSpecial() { 
    // do stuff
    return true;
}

void SomeClass::doSomething() {
    if (! checkSomethingSpecial())
        throw std::runtime_error();
    // continue on...
}

```

### Functions

In our own code, use `camelCaseNames`, not `PascalCaseNames` like in WPIlib itself.

### Macros

All caps, underscores

**Good**
```
#define THIS_IS_GOOD   1
```

**Bad**
```
#define thisIsNotGoodForAMacro   0
```

## Using Namespaces

Don't ever do `using namespace blahblah` inside a header.  If you do it, you're destined to encouter tons of conflicts and circular dependencies.