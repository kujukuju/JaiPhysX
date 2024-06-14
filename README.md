
## Jai PhysX Bindings

Nearly complete jai physx bindings. Based on the rust physx-sys bindings.


### Future Improvements

* I'd like to have a way to pass in an allocator based on jai to the actual physx cpp wrapper library so anything allocated can be freed with jai free.


### Build

I built this project piggy backing off of rust physx-sys which does a crazy autogen thing to create functions and structs.

To build it again `cargo build --release` while inside the physx-sys-extract folder. Then the lib is in `target/release/deps/`.

If you need to change the generated bindings you'll need to rerun the pxbind rust script.

NOTE: I think it needs to be regenerated and different module.jai file for different operating systems... Untested.
