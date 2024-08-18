
## Jai PhysX Bindings

Nearly complete jai physx bindings. Based on the rust physx-sys bindings.


### Future Improvements

* I'd like to have a way to pass in an allocator based on jai to the actual physx cpp wrapper library so anything allocated can be freed with jai free.

* I want to make all integer flags in the generated api actually refer to the flag it generates. It's really annoying having to casts this all.

* In the generator I have a function to force inheritance and I applied it to some things but not everything. It should work with everything.

* I'd like to see if it's possible to replace the MatNN structs with jai Matrix structs. PhysX seems to be column major while jai is row major, but the physx docs somehow implied that it'll work with row major structs which I don't understand.


### Build

I built this project piggy backing off of rust physx-sys which does a crazy autogen thing to create functions and structs.

To build it again `cargo build --release` while inside the physx-sys-extract folder. Then the lib is in `target/release/deps/`.

If you need to change the generated bindings you'll need to rerun the pxbind rust script.

On linux I'm manually swapping the cargo.toml file in physx-sys from "staticlib" to "cdylib" to try and get back a shared library since the static doesn't seem to work.
