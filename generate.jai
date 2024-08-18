
/*
Program Exception: SEGV_MAPERR
The program crashed because it tried to write to or read from an invalid memory location.
Segmentation fault. The faulty address is 0xffffffff00000020, from 0x328d00.
./main(physx_filter_shader_500000241+0x50) [0x328d00]
/root/FatGoblinsServer/libphysx_build.so(_Z22FilterShaderTrampolinejN5physx12PxFilterDataEjS0_RNS_7PxFlagsINS_10PxPairFlag4EnumEtEEPKvj+0x47) [0x79a2c484d277]
/root/FatGoblinsServer/libphysx_build.so(_ZN5physx2Sc10NPhaseCore17runOverlapFiltersEjPKNS_2Bp11AABBOverlapEPNS_12PxFilterInfoERjS8_S8_PjS9_+0xad6) [0x79a2c4e82                                                       3f6]
/root/FatGoblinsServer/libphysx_build.so(_ZN17OverlapFilterTask11runInternalEv+0x45) [0x79a2c4eaa9f5]
/root/FatGoblinsServer/libphysx_build.so(_ZN5physx2Cm4Task3runEv+0x25) [0x79a2c4aff505]
/root/FatGoblinsServer/libphysx_build.so(_ZN5physx3Ext15CpuWorkerThread7executeEv+0x57) [0x79a2c4c95237]
/root/FatGoblinsServer/libphysx_build.so(+0x26c537) [0x79a2c486c537]
/lib/x86_64-linux-gnu/libc.so.6(+0x9ca94) [0x79a2c509ca94]
/lib/x86_64-linux-gnu/libc.so.6(+0x129c3c) [0x79a2c5129c3c]




Thread 6 "main" received signal SIGSEGV, Segmentation fault.
[Switching to Thread 0x7ffff1c006c0 (LWP 31950)]
0x0000000000328b50 in physx_filter_shader_70000015a (attributes0=<optimized out>, filterData0=..., attributes1=4055889920,
    filterData1=..., pairFlags=0xffffffff00000020)
    at /mnt/c/Users/Kuju/Projects/FatGoblinsServer/../FatGoblins/src/shared/physics/SharedPhysics.jai:483
warning: 483    /mnt/c/Users/Kuju/Projects/FatGoblinsServer/../FatGoblins/src/shared/physics/SharedPhysics.jai: No such file or directory
(gdb) where
#0  0x0000000000328b50 in physx_filter_shader_70000015a (attributes0=<optimized out>, filterData0=..., attributes1=4055889920,
    filterData1=..., pairFlags=0xffffffff00000020)
    at /mnt/c/Users/Kuju/Projects/FatGoblinsServer/../FatGoblins/src/shared/physics/SharedPhysics.jai:483
#1  0x00007ffff744d277 in FilterShaderTrampoline(unsigned int, physx::PxFilterData, unsigned int, physx::PxFilterData, physx::PxFlags<physx::PxPairFlag::Enum, unsigned short>&, void const*, unsigned int) () from /root/FatGoblinsServer/libphysx_build.so
#2  0x00007ffff7a823f6 in physx::Sc::NPhaseCore::runOverlapFilters(unsigned int, physx::Bp::AABBOverlap const*, physx::PxFilterInfo*, unsigned int&, unsigned int&, unsigned int&, unsigned int*, unsigned int*) () from /root/FatGoblinsServer/libphysx_build.so
#3  0x00007ffff7aaa9f5 in OverlapFilterTask::runInternal() () from /root/FatGoblinsServer/libphysx_build.so
#4  0x00007ffff76ff505 in physx::Cm::Task::run() () from /root/FatGoblinsServer/libphysx_build.so
#5  0x00007ffff7895237 in physx::Ext::CpuWorkerThread::execute() () from /root/FatGoblinsServer/libphysx_build.so
#6  0x00007ffff746c537 in physx::(anonymous namespace)::PxThreadStart(void*) () from /root/FatGoblinsServer/libphysx_build.so
#7  0x00007ffff7c9ca94 in start_thread (arg=<optimized out>) at ./nptl/pthread_create.c:447
#8  0x00007ffff7d29c3c in clone3 () at ../sysdeps/unix/sysv/linux/x86_64/clone3.S:78
(gdb)
*/




// windows
// linux

// 00000000000000000000000000000000
// 00000000000000000000000000000000

// 00000000000000000000000000000010111111111111111111111111111111110000000000000000000000000000000000000000000000000000000000000000
// 01011111110111111111011111000000000000000000000001110100101000010000000000000000000000000000000100000000000000000000000000000000

// 00000000000000000000000000000001

// 00000000000000000000000010000000111111111111111111111111111111110000000000000000000000000000000000000000000000000000000000000000






#import "Bindings_Generator";
#import "Basic";
#import "File";
#import "String";
#import "BuildCpp";
#import "File_Utilities";

#run {
    using opts: Generate_Bindings_Options;
    // array_add(*system_include_paths, GENERATOR_DEFAULT_SYSTEM_INCLUDE_PATH);

    dependency_path: string;
    #if OS == .WINDOWS {
        dependency_path = "physx/physx-sys-extract/target/release/deps";
    } else #if OS == .LINUX {
        dependency_path = "physx/physx-build/build";
    }

    found_lib_name: string;
    // we'll now pull directly from physx-sys... because I need to compile forcing all static links or something
    for file: file_list(dependency_path) {
        filename: string;
        filename.data = file.data + (find_index_from_right(file, "/") + 1);
        filename.count = file.count - (find_index_from_right(file, "/") + 1);
        #if OS == .WINDOWS {
            if starts_with(filename, "physx") && ends_with(filename, ".lib") {
                found_lib_name = file;
            }
        } else #if OS == .LINUX {
            print("Checking %\n", filename);
            if starts_with(filename, "libphysx") && ends_with(filename, ".so") {
                found_lib_name = file;
            }
        }
    }
    if found_lib_name {
        #if OS == .WINDOWS {
            copy_file(found_lib_name, "windows/physx.lib");
        } else #if OS == .LINUX {
            copy_file(found_lib_name, "linux/libphysx.so");
        }
    }
    #if OS == .WINDOWS {
        if !file_exists("windows/physx.lib") {
            print("Physx library doesn't exist, and could not find extracted physx library to copy.\n");
            return;
        }
    } else #if OS == .LINUX {
        if !file_exists("linux/libphysx.so") {
            print("Physx library doesn't exist, and could not find extracted physx library to copy.\n");
            return;
        }
    }

    #if OS == .WINDOWS {
        array_add(*libpaths, "windows");
        array_add(*libnames, "physx");

        array_add(*include_paths, "physx/physx-sys/src/generated/x86_64-pc-windows-msvc/");
        array_add(*extra_clang_arguments, "-D_MSC_VER=1939");
        array_add(*system_include_paths, "\"C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/lib/clang/17/include\"");
    } else #if OS == .LINUX {
        array_add(*libpaths, "linux");
        array_add(*libnames, "libphysx");

        array_add(*extra_clang_arguments, "-D__clang__");
        array_add(*extra_clang_arguments, "-std=c++14");

        array_add(*include_paths, "physx/physx-sys/src/generated/unix/");
        // array_add(*source_files, "/usr/include/linux/types.h");
        array_add(*system_include_paths, "/usr/lib/llvm-18/lib/clang/18/include");
        array_add(*system_include_paths, "/usr/include/c++/14");
        // array_add(*system_include_paths, ..get_default_system_include_paths(opts.os, opts.cpu));
        // array_add(*system_include_paths, "/usr/include/c++/14/tr1");
        // array_add(*system_include_paths, "/usr/include/linux");
        // array_add(*system_include_paths, "/usr/include/x86_64-linux-gnu");
    }

    array_add(*include_paths, "physx/physx-sys/src/");
    array_add(*include_paths, "physx/physx-sys/physx/physx/include/");

    array_add(*extra_clang_arguments, "-DNDEBUG");

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
        decl.decl_flags = .OMIT_FROM_OUTPUT;
        if decl.parent {
            remove_self_and_parents(decl.parent);
        }
    }

    opts.visitor = (decl: *Declaration, parent_decl: *Declaration) -> Declaration_Visit_Result {
        filename_clang := clang_getFileName(decl.location.file);
        defer clang_disposeString(filename_clang);
        filename := to_lower_copy_new(to_string(clang_getCString(filename_clang)),, temp);

        // things to allow through no matter what, but it won't recurse to these necessarily since I stop in some cases
        okay_things :: string.[
            "PxSimulationFilterShader",
        ];

        for okay_thing: okay_things {
            if decl.name == okay_thing {
                return .RECURSE;
            }
        }

        if contains(filename, "physx/physx/") {
            if contains(decl.name, "PxSimulationFilterShader") {
                print("\n\n\n\nFOUND SOMETHING... %\n\n\n\n\n", decl.*);
            }
            decl.decl_flags = .OMIT_FROM_OUTPUT;
            return .RECURSE;
        }

        for conv: convlist {
            if conv[0] == decl.name {
                decl.output_name = conv[1];
                decl.decl_flags = .OMIT_FROM_OUTPUT;
                return .RECURSE;
            }
        }

        if contains(filename, "physx_api") {
            if starts_with(decl.foreign_name, "?") || starts_with(decl.foreign_name, "_") {
                remove_self_and_parents(decl);
                return .STOP;
            }
        }

        // the names of the things to remove from the bindings, before the _mut and _1 stuff gets stripped
        bad_things :: string.[
            "PxMat44_new_1",
            "PxMat44_new_2",
            "PxMat33_new_1",
            "PxMat33_new_2",
            "PxGeometryHolder_any",
            "PxGeometryHolder_sphere",
            "PxGeometryHolder_plane",
            "PxGeometryHolder_capsule",
            "PxGeometryHolder_box",
            "PxGeometryHolder_convexMesh",
            "PxGeometryHolder_tetMesh",
            "PxGeometryHolder_triangleMesh",
            "PxGeometryHolder_heightField",
            "PxGeometryHolder_particleSystem",
            "PxGeometryHolder_hairSystem",
            "PxGeometryHolder_custom",
            "PxSoftBodyMesh_getCollisionMesh",
            "PxSoftBodyMesh_getSimulationMesh",
            "PxSoftBodyMesh_getSoftBodyAuxData",
            "PxCollisionTetrahedronMeshData_getMesh",
            "PxCollisionTetrahedronMeshData_getData",
            "PxTransform_new",
            "PxQuat_new",
        ];

        for bad_thing: bad_things {
            if decl.name == bad_thing {
                decl.decl_flags = .OMIT_FROM_OUTPUT;
                return .STOP;
            }
        }

        if ends_with(decl.name, "_mut") {
            output_name := decl.name;
            output_name.count -= 4;
            decl.output_name = output_name;
        }
        if ends_with(decl.name, "_mut_1") || ends_with(decl.name, "_mut_2") {
            output_name := decl.name;
            output_name.count -= 6;
            decl.output_name = output_name;
        }
        if ends_with(decl.name, "_1") || ends_with(decl.name, "_2") || ends_with(decl.name, "_3") || ends_with(decl.name, "_4") || ends_with(decl.name, "_5") || ends_with(decl.name, "_6") || ends_with(decl.name, "_7") || ends_with(decl.name, "_8") || ends_with(decl.name, "_9") {
            output_name := decl.name;
            output_name.count -= 2;
            decl.output_name = output_name;
        }
        if ends_with(decl.name, "_10") {
            output_name := decl.name;
            output_name.count -= 3;
            decl.output_name = output_name;
        }
        if starts_with(decl.name, "physx_") {
            output_name := decl.name;
            output_name.data += 6;
            output_name.count -= 6;
            decl.output_name = output_name;
        }
        if starts_with(decl.name, "phys_") {
            output_name := decl.name;
            output_name.data += 5;
            output_name.count -= 5;
            decl.output_name = output_name;
        }

        return .RECURSE;
    };

    header = read_entire_file("generate_enums.jai");

    footer = #string ___jai
#import "Math";
#import "Socket";
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
    // change the function mapping names to something that won't get replaced
    output = replace_and_free(output, "\"physx_", "\"PHYSX_");
    output = replace_and_free(output, "physx.", "physx_");
    output = replace_and_free(output, "physx_", "");
    // change the function mapping names back
    output = replace_and_free(output, "\"PHYSX_", "\"physx_");
    output = replace_and_free(output, "__pod", "");
    output = replace_and_free(output, "_pod", "");
    output = replace_and_free(output, "context:", "context_pod:");
    output = force_inheritance(output, "#place structgen_pad0; #as geometry: PxGeometry = ---;", .[
        "PxBoxGeometry",
        "PxCapsuleGeometry",
        "PxConvexMeshGeometry",
        "PxSphereGeometry",
        "PxPlaneGeometry",
        "PxTriangleMeshGeometry",
        "PxHeightFieldGeometry",
        "PxParticleSystemGeometry",
        "PxHairSystemGeometry",
        "PxTetrahedronMeshGeometry",
        "PxCustomGeometry",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as actor: PxActor = ---;", .[
        "PxRigidActor",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as rigid_actor: PxRigidActor = ---;", .[
        "PxRigidStatic",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as rigid_actor: PxRigidActor = ---;", .[
        "PxRigidBody",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as rigid_body: PxRigidBody = ---;", .[
        "PxRigidDynamic",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as rigid_body: PxRigidBody = ---;", .[
        "PxArticulationLink",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as base: PxBase = ---;", .[
        "PxRefCounted",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as ref_counted: PxRefCounted = ---;", .[
        "PxBaseMaterial",
        "PxConvexMesh",
        "PxHeightField",
        "PxShape",
        "PxSoftBodyAuxData",
        "PxSoftBodyMesh",
        "PxTetrahedronMesh",
        "PxTriangleMesh",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as base_material: PxBaseMaterial = ---;", .[
        "PxMaterial",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as controller_desc: PxControllerDesc = ---;", .[
        "PxBoxControllerDesc",
        "PxCapsuleControllerDesc",
    ]);
    output = force_inheritance(output, "#place vtable_; #as controller: PxController = ---;", .[
        "PxBoxController",
        "PxCapsuleController",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as base: PxBase = ---;", .[
        "PxJoint",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as joint: PxJoint = ---;", .[
        "PxContactJoint",
        "PxD6Joint",
        "PxDistanceJoint",
        "PxFixedJoint",
        "PxPrismaticJoint",
        "PxRevoluteJoint",
        "PxSphericalJoint",
        "PxGearJoint",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as output_stream: PxOutputStream = ---;", .[
        "PxDefaultMemoryOutputStream",
    ]);
    output = force_inheritance(output, "#place structgen_pad0; #as input_stream: PxInputStream = ---;", .[
        "PxDefaultMemoryInputData",
    ]);

    #if OS == .WINDOWS {
        write_entire_file("module_windows.jai", output);
    } else #if OS == .LINUX {
        write_entire_file("module_linux.jai", output);
    }
}

replace_and_free :: (input: string, old: string, new: string) -> string {
    output := replace(input, old, new);
    free(input);
    return output;
}

force_inheritance :: (input: string, place: string, struct_names: [] string) -> string {
    current := input;

    struct_part :: " :: struct {";
    for struct_name: struct_names {
        index := 0;
        while index < current.count - struct_name.count - struct_part.count {
            potential_name: string;
            potential_name.data = current.data + index;
            potential_name.count = struct_name.count;

            if potential_name == struct_name {
                potential_struct_part: string;
                potential_struct_part.data = potential_name.data + potential_name.count;
                potential_struct_part.count = struct_part.count;

                if potential_struct_part == struct_part {
                    last_colon := index;
                    while current[index] != #char "}" {
                        if current[index] == #char ";" {
                            last_colon = index;
                        }

                        index += 1;
                    }

                    first_string: string;
                    first_string.data = current.data;
                    first_string.count = last_colon + 1;
                    second_string: string;
                    second_string.data = current.data + (last_colon + 1);
                    second_string.count = current.count - (last_colon + 1);

                    old_current := current;
                    current = sprint("%1\n%2%3", first_string, place, second_string);
                    free(old_current);

                    continue struct_name;
                }
            }

            index += 1;
        }
    }

    return current;
}
