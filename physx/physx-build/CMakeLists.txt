
cmake_minimum_required(VERSION 3.25)
project(physx)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DDISABLE_CUDA_PHYSX)
add_definitions(-DPX_SUPPORT_PVD)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DNDEBUG)
endif()

if(WIN32)
    include_directories(${CMAKE_SOURCE_DIR}/../physx-sys/src/generated/x86_64-pc-windows-msvc)
elseif(UNIX AND NOT APPLE)
    include_directories(${CMAKE_SOURCE_DIR}/../physx-sys/src/generated/unix)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_definitions(-DPX_CLANG)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_definitions(-DPX_GCC)
endif()

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        add_definitions(-DPX_WIN64)
    else()
        add_definitions(-DPX_WIN32)
    endif()
elseif(UNIX AND NOT APPLE)
    add_definitions(-DPX_LINUX)
endif()

add_definitions(-DPX_PHYSX_CORE_EXPORTS)
add_definitions(-DPX_PHYSX_COMMON_EXPORTS)
add_definitions(-DPX_PHYSX_COOKING_EXPORTS)
add_definitions(-DPX_PHYSX_FOUNDATION_EXPORTS)

include_directories(
        ${CMAKE_SOURCE_DIR}/../physx-sys/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/include

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/foundation/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/common/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/common/src
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/filebuf/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/pxshared/include

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/fastxml/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/fastxml/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/task/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/task/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/foundation

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/api/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/common/include/collision
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/common/include/pipeline
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/common/include/utils
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/software/include

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevelaabb/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevelaabb/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowleveldynamics/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowleveldynamics/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowleveldynamics/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowleveldynamics/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxvehicle/src
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxvehicle/src/physxmetadata/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxvehicle/src/physxmetadata/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/extensions/src
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/File
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/Xml
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/Binary
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/tet
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/extensions/include

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxcharacterkinematic/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/common/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/common/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/ccd
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/common
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/contact
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/convex
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/cooking
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/distance
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/gjk
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/hf
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/intersection
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/mesh
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/pcm
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/sweep

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxcooking/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/pvd/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/pvd/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/core/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/core/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physx/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/scenequery/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/scenequery/src

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/simulationcontroller/include
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/simulationcontroller/src
)

file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/../physx-sys/src/physx_api.cpp)


if(WIN32)
    file(GLOB OS_SOURCES ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/foundation/windows/*.cpp)
    list(APPEND SOURCES ${OS_SOURCES})
elseif(UNIX AND NOT APPLE)
    file(GLOB OS_SOURCES ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/foundation/unix/*.cpp)
    list(APPEND SOURCES ${OS_SOURCES})
endif()

file(GLOB FINAL_SOURCES
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/foundation/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/api/src/px_globals.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/common/src/pipeline/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevel/software/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/fastxml/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/task/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowlevelaabb/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/lowleveldynamics/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxvehicle/src/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxvehicle/src/physxmetadata/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtBroadPhase.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtCollection.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtContactJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtConvexMeshExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtCpuWorkerThread.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtCustomGeometryExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtCustomSceneQuerySystem.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtD6Joint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtD6JointCreate.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtDefaultCpuDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtDefaultErrorCallback.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtDefaultSimulationFilterShader.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtDefaultStreams.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtDistanceJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtExtensions.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtFixedJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtGearJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtGjkQueryExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtMetaData.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtPrismaticJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtPvd.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtPxStringTable.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRackAndPinionJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRaycastCCD.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRemeshingExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRevoluteJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRigidActorExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtRigidBodyExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSampling.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSceneQueryExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSceneQuerySystem.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSimpleFactory.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSmoothNormals.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSoftBodyExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSphericalJoint.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSqManager.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtSqQuery.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtTetMakerExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtTetrahedronMeshExt.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/ExtTriangleMeshExt.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/extensions/src/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/File/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/Xml/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/serialization/Binary/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxextensions/src/tet/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxcharacterkinematic/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/common/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/ccd/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/common/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/contact/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/convex/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/cooking/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/distance/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/gjk/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/hf/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/intersection/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/mesh/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/pcm/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/geomutils/src/sweep/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxcooking/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/pvd/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physxmetadata/core/src/*.cpp
        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/immediatemode/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/physx/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/scenequery/src/*.cpp

        ${CMAKE_SOURCE_DIR}/../physx-sys/physx/physx/source/simulationcontroller/src/*.cpp
)
list(APPEND SOURCES ${FINAL_SOURCES})

#if(WIN32)
#    list(APPEND SOURCES )
#elseif(UNIX AND NOT APPLE)
#    list(APPEND SOURCES )
#endif()

add_library(physx SHARED ${SOURCES})
