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

        if contains(filename, "physx_api") {
            if starts_with(decl.foreign_name, "?") {
                remove_self_and_parents(decl);
                return .STOP;
            }
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
    ___jai

    string_builder := generate_bindings(opts, true);
    output := builder_to_string(*string_builder);
    output = replace_and_free(output, "PxReal", "float");
    output = replace_and_free(output, "PxF32", "float");
    output = replace_and_free(output, "double", "float64");
    output = replace_and_free(output, "int8_t", "s8");
    output = replace_and_free(output, "char", "s8");
    output = replace_and_free(output, "PxI8", "s8");
    output = replace_and_free(output, "uint8_t", "u8");
    output = replace_and_free(output, "unsigned char", "u8");
    output = replace_and_free(output, "PxU8", "u8");
    output = replace_and_free(output, "int16_t", "s16");
    output = replace_and_free(output, "short", "s16");
    output = replace_and_free(output, "PxI16", "s16");
    output = replace_and_free(output, "uint16_t", "u16");
    output = replace_and_free(output, "unsigned short", "u16");
    output = replace_and_free(output, "PxU16", "u16");
    output = replace_and_free(output, "int32_t", "s32");
    output = replace_and_free(output, "int", "s32");
    output = replace_and_free(output, "PxI32", "s32");
    output = replace_and_free(output, "uint32_t", "u32");
    output = replace_and_free(output, "unsigned int", "u32");
    output = replace_and_free(output, "PxU32", "u32");
    output = replace_and_free(output, "int64_t", "s64");
    output = replace_and_free(output, "long", "s64");
    output = replace_and_free(output, "PxI64", "s64");
    output = replace_and_free(output, "uint64_t", "u64");
    output = replace_and_free(output, "unsigned long", "u64");
    output = replace_and_free(output, "PxU64", "u64");
    output = replace_and_free(output, "size_t", "u64");
    output = replace_and_free(output, "PxVec3", "Vector3");
    output = replace_and_free(output, "PxVec4", "Vector4");
    output = replace_and_free(output, "PxMat33", "Matrix3");
    output = replace_and_free(output, "PxMat44", "Matrix4");
    output = replace_and_free(output, "PxAgain", "bool");
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
    output = replace_and_free(output, "physx.Vector3", "Vector3");
    output = replace_and_free(output, "physx.Vector4", "Vector4");
    output = replace_and_free(output, "physx.Matrix3", "Matrix3");
    output = replace_and_free(output, "physx.Matrix4", "Matrix4");
    output = replace_and_free(output, "physx.bool", "bool");
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
