
## Build

The only thing I had to do to get this building was comment out:

Everything that starts with `PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF` and `PX_COMPILE_TIME_ASSERT((PX_OFFSET_OF`.

This is because, I guess, you're not allowed to do this specific type of offset inside a compile time assert in clang? I don't know...

### Module Fixes

// for syntax highlighting

`/**`
->
`/*`

// for syntax highlighting

`\/[\*]+\/`
->
`//`

`
 :: struct {
    minus1w: [4] PxF32;
}
`
->
`
PxF32 :: struct {
    minus1w: [4] PxF32;
}
`

`340282346638528859000000000000000000000.0`
->
`FLOAT32_MAX`

`113427448879509619900000000000000000000.0`
->
`FLOAT32_MAX`

`PxMeshGeometryFlags = PxFlags.{}`
->
`PxMeshGeometryFlags = PxMeshGeometryFlags.{}`

// not sure sure about this one

`PxArray :: struct(T: Type, Alloc: Type)`
->
`PxArray :: struct(T: Type, Alloc: Type = PxAllocatorTraits(T))`

`PxHitFlags = PxFlags`
->
`PxHitFlags = PxHitFlags`

`PxTransform = PxTransformT`
->
`PxTransform = PxTransform`

`PxTransform.{xx PxIDENTITY.PxIdentity}`
->
`PxTransform.{}`