# LieGroups

A small utility library for some group operations on SO3 and SE3 (and eventually more). 
The library is header-only.


Let G be a group, g the lie algebra, and T the tangent space. 
The main operations are
- Exp : T -> M
- Log : M -> T
- action: M x v -> v

here v is the vector space the group element can act on. 

## Usage

See the `examples/main.cpp` or `tests/liegroup_test.cpp`  for some examples on how to use the library. 

## Building
Its a standard cmake library:

```
mkdir build
cd build
cmake ..
make 
```

to test, 
```
cd build
make 
ctest --output-on-failure
```

If the library is installed with the default directories, you can simply 
```
#include <liegroups/liegroups.hpp>
```
in any source file to start using this library. 
You will also need to use Eigen, so in your `CMakeLists.txt` make sure you have
```
find_package(Eigen3 3.4 REQUIRED)

...

add_executable( ... )
target_link_libraries( ...
  Eigen3:Eigen
)

## Notes
There is a pre-commit hook to ensure that the code is properly formatted. Run
```
clang-format -i --verbose **/*.hpp **/*.cpp
```
to ensure formatting styles before committing. 
