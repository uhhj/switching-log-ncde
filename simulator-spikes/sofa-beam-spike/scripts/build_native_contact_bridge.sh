#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SPIKE="$ROOT/simulator-spikes/sofa-beam-spike"
PLUGIN_SRC="$SPIKE/native-contact-bridge"
PLUGIN_BUILD="$PLUGIN_SRC/build"
SOFA_ROOT="${SOFA_ROOT:-/root/workspace/third_party/SOFA_v26.06.00_Linux}"

[[ -x "$SOFA_ROOT/bin/runSofa" ]] || { echo "missing runSofa under $SOFA_ROOT" >&2; exit 2; }
[[ -d "$SOFA_ROOT/include" ]] || { echo "SOFA binary has no include/ development files" >&2; exit 3; }
[[ -d "$SOFA_ROOT/lib/cmake" ]] || { echo "SOFA binary has no lib/cmake development files" >&2; exit 4; }

build() {
    rm -rf "$PLUGIN_BUILD"
    cmake -S "$PLUGIN_SRC" -B "$PLUGIN_BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$SOFA_ROOT/lib/cmake"
    cmake --build "$PLUGIN_BUILD" --parallel 2
}

if command -v cmake >/dev/null 2>&1 && command -v ninja >/dev/null 2>&1; then
    build
else
    # The validated runner is intentionally runtime-only. This short-lived
    # Ubuntu 24.04 container supplies only the C++/CMake toolchain; it does
    # not build SOFA, SofaPython3, or BeamAdapter.
    docker run --rm --network host \
        -v "$ROOT:$ROOT" \
        -v "$SOFA_ROOT:$SOFA_ROOT:ro" \
        -w "$ROOT" ubuntu:24.04 bash -lc \
        'apt-get update >/dev/null && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends g++ cmake ninja-build libboost-dev libeigen3-dev >/dev/null && SOFA_ROOT="$1" "$2"' \
        bash "$SOFA_ROOT" "$SPIKE/scripts/build_native_contact_bridge.sh"
fi

LIB="$PLUGIN_BUILD/lib/libNativeContactBridge.so"
[[ -f "$LIB" ]] || { echo "plugin library not produced: $LIB" >&2; exit 6; }
echo "$LIB"
