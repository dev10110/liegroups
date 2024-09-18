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


