#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$project_dir/build-release"
cmake_args=(-S "$project_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON)

if [[ "$(uname -s)" == "Darwin" ]]; then
    # Use a matching compiler and SDK.
    cmake_args+=("-DCMAKE_CXX_COMPILER=$(xcrun --find clang++)")
    cmake_args+=("-DCMAKE_OSX_SYSROOT=$(xcrun --sdk macosx --show-sdk-path)")
    if command -v brew >/dev/null 2>&1; then
        cmake_args+=("-DCMAKE_PREFIX_PATH=$(brew --prefix)")
    fi
fi

cmake "${cmake_args[@]}"
cmake --build "$build_dir" --parallel 4

if [[ "${1:-}" == "--build-only" ]]; then
    exit 0
elif [[ "${1:-}" == "--test" ]]; then
    ctest --test-dir "$build_dir" --output-on-failure
    exit 0
fi

if [[ "$(uname -s)" == "Darwin" ]]; then
    exec "$build_dir/RailLab.app/Contents/MacOS/RailLab" "$@"
else
    exec "$build_dir/RailLab" "$@"
fi
