#!/usr/bin/env bash
set -euo pipefail

PRESET="${1:-linux-system}"
SKIP_TESTS="${SKIP_TESTS:-0}"

case "${PRESET}" in
  linux-system|linux-fetchcontent|linux-production)
    ;;
  *)
    echo "Unknown preset: ${PRESET}" >&2
    echo "Use one of: linux-system, linux-fetchcontent, linux-production" >&2
    exit 2
    ;;
esac

command -v cmake >/dev/null 2>&1 || {
  echo "CMake was not found in PATH." >&2
  exit 1
}

command -v ninja >/dev/null 2>&1 || {
  echo "Ninja was not found in PATH." >&2
  exit 1
}

cmake --preset "${PRESET}"
cmake --build --preset "${PRESET}"

if [[ "${SKIP_TESTS}" != "1" && "${PRESET}" != "linux-production" ]]; then
  ctest --preset "${PRESET}"
fi
