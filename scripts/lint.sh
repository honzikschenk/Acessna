#!/bin/sh
# Runs clang-tidy on code
# Arguments go to run-clang-tidy
set -e

cd "$(dirname "$0")/.."

RUN_CLANG_TIDY="${RUN_CLANG_TIDY:-$(command -v run-clang-tidy || echo /opt/homebrew/opt/llvm/bin/run-clang-tidy)}"
if [ ! -x "$RUN_CLANG_TIDY" ]; then
  echo "run-clang-tidy not found. Install LLVM (brew install llvm) or set RUN_CLANG_TIDY" >&2
  exit 1
fi
CLANG_TIDY="$(dirname "$RUN_CLANG_TIDY")/clang-tidy"

if [ ! -f build/compile_commands.json ]; then
  echo "build/compile_commands.json missing. Run: cmake -S . -B build" >&2
  exit 1
fi

EXTRA=""
if command -v xcrun >/dev/null 2>&1; then
  EXTRA="-extra-arg=-isysroot$(xcrun --show-sdk-path)"
fi

exec "$RUN_CLANG_TIDY" -quiet -p build -clang-tidy-binary "$CLANG_TIDY" $EXTRA "$@" \
  "$(pwd)/(src|tests)/.*\.cpp$"
