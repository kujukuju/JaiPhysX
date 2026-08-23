#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD_DIR="$ROOT_DIR/.build/macos-universal"

cmake \
    -S "$ROOT_DIR/physx/physx-build" \
    -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    '-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64' \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 \
    -DCMAKE_C_COMPILER=/usr/bin/clang \
    -DCMAKE_CXX_COMPILER=/usr/bin/clang++

cmake --build "$BUILD_DIR" --config Release --parallel

mkdir -p "$ROOT_DIR/macos"
cp "$BUILD_DIR/libphysx.dylib" "$ROOT_DIR/macos/libphysx.dylib"
