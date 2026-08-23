
#if OS == .WINDOWS {
    #load "module_windows.jai";
} else #if OS == .LINUX {
    #load "module_linux.jai";
} else #if OS == .MACOS {
    #load "module_macos.jai";
} else {
    #assert false "Unsupported PhysX OS.\n";
}