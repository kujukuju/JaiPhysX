#import "Bindings_Generator";
#import "Basic";
#import "File";
#import "String";
#import "BuildCpp";
#import "File_Utilities";

#run {
    using opts: Generate_Bindings_Options;
    // array_add(*system_include_paths, GENERATOR_DEFAULT_SYSTEM_INCLUDE_PATH);
    // array_add(*system_include_paths, ..get_default_system_include_paths(opts.os, opts.cpu));

    array_add(*libpaths, "windows");

    array_add(*libnames, "physx_sys");

    array_add(*include_paths, "physx/physx-sys/src/generated/x86_64-pc-windows-msvc/");
    array_add(*include_paths, "physx/physx-sys/src/");
    array_add(*include_paths, "physx/physx-sys/physx/physx/include/");

    array_add(*extra_clang_arguments, "-DNDEBUG");
    array_add(*extra_clang_arguments, "-D_MSC_VER=1939");

    array_add(*system_include_paths, "\"C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/lib/clang/17/include\"");

    array_add(*source_files, "physx/physx-sys/physx/physx/include/PxPhysicsAPI.h");

    array_add(*source_files, "physx/physx-sys/src/physx_api.cpp");

    array_add(*extra_clang_arguments, "-x", "c++");
    generate_compile_time_struct_checks = false;

    convlist :: [2] string.[
        .["physx_PxVec2", "Vector2"],
        .["physx_PxVec3", "Vector3"],
        .["physx_PxVec4", "Vector4"],
        .["physx_Vector2", "Vector2"],
        .["physx_Vector3", "Vector3"],
        .["physx_Vector4", "Vector4"],
        .["physx_PxQuat", "Quaternion"],
        .["physx_PxPlane", "Plane3"],
        .["PxReal", "float"],
        .["PxF32", "float"],
        .["double", "float64"],
        .["int8_t", "s8"],
        .["char", "s8"],
        .["PxI8", "s8"],
        .["uint8_t", "u8"],
        .["unsigned char", "u8"],
        .["PxU8", "u8"],
        .["int16_t", "s16"],
        .["short", "s16"],
        .["PxI16", "s16"],
        .["uint16_t", "u16"],
        .["unsigned short", "u16"],
        .["PxU16", "u16"],
        .["int32_t", "s32"],
        .["int", "s32"],
        .["PxI32", "s32"],
        .["uint32_t", "u32"],
        .["unsigned int", "u32"],
        .["PxU32", "u32"],
        .["int64_t", "s64"],
        .["long", "s64"],
        .["PxI64", "s64"],
        .["uint64_t", "u64"],
        .["unsigned long", "u64"],
        .["PxU64", "u64"],
        .["size_t", "u64"],
        .["PxVec3", "Vector3"],
        .["PxVec4", "Vector4"],
        .["PxAgain", "bool"],
    ];

    remove_self_and_parents :: (decl: *Declaration) {
        print("removing: %\n%\n\n", decl.*, decl.library.*);

        decl.decl_flags = .OMIT_FROM_OUTPUT;
        if decl.parent {
            remove_self_and_parents(decl.parent);
        }
    }

    opts.visitor = (decl: *Declaration, parent_decl: *Declaration) -> Declaration_Visit_Result {
        filename_clang := clang_getFileName(decl.location.file);
        defer clang_disposeString(filename_clang);
        filename := to_lower_copy_new(to_string(clang_getCString(filename_clang)),, temp);

        if contains(filename, "physx/physx/") {
            decl.decl_flags = .OMIT_FROM_OUTPUT;
            return .STOP;
        }

        for conv: convlist {
            if conv[0] == decl.name {
                decl.output_name = conv[1];
                decl.decl_flags = .OMIT_FROM_OUTPUT;
                print("Found:\n%\n%\n\n", decl.*, decl.expression.*);
            }
        }

        if contains(filename, "physx_api") {
            if starts_with(decl.foreign_name, "?") {
                remove_self_and_parents(decl);
                return .STOP;
            }
        }

        if ends_with(decl.name, "_mut") {
            print("Replacing foreign name for: %\n", decl.*);
            decl.output_name = replace(decl.name, "_mut", "");
            // decl.foreign_name = decl.name;
        }
        if ends_with(decl.name, "_mut_1") {
            print("Replacing foreign name for: %\n", decl.*);
            decl.output_name = replace(decl.name, "_mut_1", "");
            // decl.foreign_name = decl.name;
        }
        if ends_with(decl.name, "_mut_2") {
            print("Replacing foreign name for: %\n", decl.*);
            decl.output_name = replace(decl.name, "_mut_2", "");
            // decl.foreign_name = decl.name;
        }

        return .RECURSE;
    };

    header = #string ___jai
physx_PxPairFlags :: enum_flags u16 {
    SolveContact :: 1 << 0;
    ModifyContacts :: 1 << 1;
    NotifyTouchFound :: 1 << 2;
    NotifyTouchPersists :: 1 << 3;
    NotifyTouchLost :: 1 << 4;
    NotifyTouchCcd :: 1 << 5;
    NotifyThresholdForceFound :: 1 << 6;
    NotifyThresholdForcePersists :: 1 << 7;
    NotifyThresholdForceLost :: 1 << 8;
    NotifyContactPoints :: 1 << 9;
    DetectDiscreteContact :: 1 << 10;
    DetectCcdContact :: 1 << 11;
    PreSolverVelocity :: 1 << 12;
    PostSolverVelocity :: 1 << 13;
    ContactEventPose :: 1 << 14;
    NextFree :: 1 << 15;
    ContactDefault :: SolveContact | DetectDiscreteContact;
    TriggerDefault :: NotifyTouchFound | NotifyTouchLost | DetectDiscreteContact;
}

physx_PxDefaultCpuDispatcherWaitForWorkMode :: enum s32 {
    WaitForWork :: 0;
    YieldThread :: 1;
    YieldProcessor :: 2;
}

physx_PxConvexMeshCookingType :: enum s32 {
    Quickhull :: 0;
}

physx_PxConvexFlag :: enum_flags s32 {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    E16BitIndices :: 1;
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    ComputeConvex :: 2;
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    CheckZeroAreaTriangles :: 4;
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    QuantizeInput :: 8;
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    DisableMeshValidation :: 16;
    /// Enables plane shifting vertex limit algorithm.
    ///
    /// Plane shifting is an alternative algorithm for the case when the computed hull has more vertices
    /// than the specified vertex limit.
    ///
    /// The default algorithm computes the full hull, and an OBB around the input vertices. This OBB is then sliced
    /// with the hull planes until the vertex limit is reached.The default algorithm requires the vertex limit
    /// to be set to at least 8, and typically produces results that are much better quality than are produced
    /// by plane shifting.
    ///
    /// When plane shifting is enabled, the hull computation stops when vertex limit is reached. The hull planes
    /// are then shifted to contain all input vertices, and the new plane intersection points are then used to
    /// generate the final hull with the given vertex limit.Plane shifting may produce sharp edges to vertices
    /// very far away from the input cloud, and does not guarantee that all input vertices are inside the resulting
    /// hull.However, it can be used with a vertex limit as low as 4.
    PlaneShifting :: 32;
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    FastInertiaComputation :: 64;
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    GpuCompatible :: 128;
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    ShiftVertices :: 256;
}

PxConvexMeshCookingResult :: enum s32 {
    /// Convex mesh cooking succeeded.
    Success :: 0;
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    ZeroAreaTestFailed :: 1;
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PolygonsLimitReached :: 2;
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure :: 3;
}
    ___jai

    footer = #string ___jai
#import "Math";
    ___jai

    string_builder := generate_bindings(opts, true);
    output := builder_to_string(*string_builder);
    output = replace_and_free(output, "physx.PxFilterObjectAttributes", "u32");
    output = replace_and_free(output, "physx.PxFilterObjectAttributes", "u32");
    output = replace_and_free(output, "physx.PxFilterObjectAttributes", "u32");
    output = replace_and_free(output, "physx.float", "float");
    output = replace_and_free(output, "physx.float64", "float64");
    output = replace_and_free(output, "physx.s8", "s8");
    output = replace_and_free(output, "physx.u8", "u8");
    output = replace_and_free(output, "physx.s16", "s16");
    output = replace_and_free(output, "physx.u16", "u16");
    output = replace_and_free(output, "physx.s32", "s32");
    output = replace_and_free(output, "physx.u32", "u32");
    output = replace_and_free(output, "physx.s64", "s64");
    output = replace_and_free(output, "physx.u64", "u64");
    output = replace_and_free(output, "physx.PxVec2", "Vector3");
    output = replace_and_free(output, "physx.Vector3", "Vector3");
    output = replace_and_free(output, "physx.Vector4", "Vector4");
    output = replace_and_free(output, "physx.Matrix3", "Matrix3");
    output = replace_and_free(output, "physx.Matrix4", "Matrix4");
    output = replace_and_free(output, "physx.bool", "bool");
    output = replace_and_free(output, "physx.PxQuat", "Quaternion");
    output = replace_and_free(output, "physx.PxPlane", "Plane3");
    output = replace_and_free(output, "physx.PxI16", "s16");
    output = replace_and_free(output, "physx.PxU16", "u16");
    output = replace_and_free(output, "physx.PxI32", "s32");
    output = replace_and_free(output, "physx.PxU32", "u32");
    output = replace_and_free(output, "physx.PxAgain", "bool");
    output = replace_and_free(output, "physx.", "physx_");
    output = replace_and_free(output, "__pod", "");
    output = replace_and_free(output, "_pod", "");
    output = replace_and_free(output, "context:", "context_pod:");

    write_entire_file("module.jai", output);
}

replace_and_free :: (input: string, old: string, new: string) -> string {
    output := replace(input, old, new);
    free(input);
    return output;
}
