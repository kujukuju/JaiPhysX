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

    array_add(*libnames, "PhysXFoundation_static");
    array_add(*libnames, "PhysXCharacterKinematic_static");
    array_add(*libnames, "PhysXCommon_static");
    array_add(*libnames, "PhysXCooking_static");
    // array_add(*libnames, "PhysXPvdSDK_static");
    array_add(*libnames, "PhysX_static");
    // array_add(*libnames, "PhysXVehicle_static");
    // array_add(*libnames, "PhysXExtensions_static");
    // array_add(*libnames, "PhysXVehicle2_static");

    array_add(*include_paths, "Physx/physx/include");
    array_add(*extra_clang_arguments, "-DNDEBUG");
    // array_add(*extra_clang_arguments, "-DPX_SSE2=1");
    // array_add(*extra_clang_arguments, "-msse2", "-mavx", "-DPX_SUPPORT_PVD=0", "-DPX_SUPPORT_OMNI_PVD=0");
    // array_add(*extra_clang_arguments, "-DCOMPILE_VECTOR_INTRINSICS=1", "-DPX_SUPPORT_SIMD=1");

    // array_add(*extra_clang_arguments, "-U__clang__");
    // array_add(*extra_clang_arguments, "-D__MSVC__");
    // array_add(*extra_clang_arguments, "--target=x86_64-pc-windows-msvc");
    array_add(*extra_clang_arguments, "-D_MSC_VER=1939");
    // array_add(*extra_clang_arguments, "-U_MSC_VER");

    // array_add(*system_include_paths, "C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/ucrt");
    // array_add(*system_include_paths, "C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/um");
    // array_add(*system_include_paths, "C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/shared");
    // array_add(*system_include_paths, "\"C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.31.31103/include\"");
    array_add(*system_include_paths, "\"C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/lib/clang/17/include\"");

    array_add(*source_files, "PhysX/physx/include/PxPhysicsAPI.h");

    array_add(*extra_clang_arguments, "-x", "c++");
    generate_compile_time_struct_checks = false;

    opts.visitor = (decl: *Declaration, parent_decl: *Declaration) -> Declaration_Visit_Result {
        // if decl.name == "Vector3" {
        //     decl.decl_flags |= .OMIT_FROM_OUTPUT;
        //     return .STOP;
        // }
        return .RECURSE;
    };

    generate_bindings(opts, "module.jai");
}

// get_cmake_cache_settings :: (settings: [] string) -> [] string {
//     cache_file: string;
//     for file: file_list("Physx/physx/compiler/") {
//         if ends_with(file, "CMakeCache.txt") {
//             assert(!cache_file, "Found multiple cache files.\n");
//             cache_file = file;
//         }
//     }
//     assert(!!cache_file, "Could not find build cache file.\n");

//     cache_contents, cache_read := read_entire_file(cache_file);
//     assert(cache_read);

//     // memory leak for fun

//     values: [..] string;

//     cache_lines := split(cache_contents, "\n");
//     for cache_line {
//         cache_parts := split(cache_line, "=");
//         for cache_part: cache_parts {
//             for setting: settings {
//                 if cache_part[0] == setting {
//                     array_add(*values, cache_part[1]);
//                 }
//             }
//         }
//     }

//     return values;
// }
