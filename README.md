
## Jai PhysX Bindings

Nearly complete jai physx bindings. Based on the rust physx-sys bindings.


### Future Improvements

* I'd like to have a way to pass in an allocator based on jai to the actual physx cpp wrapper library so anything allocated can be freed with jai free.

* I want to make all integer flags in the generated api actually refer to the flag it generates. It's really annoying having to casts this all.

* In the generator I have a function to force inheritance and I applied it to some things but not everything. It should work with everything.

* I'd like to see if it's possible to replace the MatNN structs with jai Matrix structs. PhysX seems to be column major while jai is row major, but the physx docs somehow implied that it'll work with row major structs which I don't understand.


### Build

The native wrapper now builds directly from C++ with `physx/physx-build/CMakeLists.txt`; Rust and Cargo are not part of the native build.

On macOS, run:

```sh
./build_macos.sh
```

This produces a Release, macOS 11+, universal `arm64` + `x86_64` dylib at `macos/libphysx.dylib`. The build compiles the retained `physx-sys` generated C++ API and validates its 323 generated POD sizes with native `static_assert` checks.

To regenerate Jai declarations, run `jai generate.jai` from this directory. macOS generation writes `module_macos_new.jai` for ABI checking and manual comparison; do not overwrite the reviewed platform modules without preserving their inheritance and padding corrections.
