
## Build

I built this project piggy backing off of rust physx-sys which does a crazy autogen thing to create functions and structs.

To build it again `cargo build --release` while inside the physx-sys-extract folder. Then the lib is in `target/release/deps/`.

NOTE: I think it needs to be regenerated and different module.jai file for different operating systems... Untested.
