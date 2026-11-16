#!/bin/sh
# Runs clang-format on code
set -e

cd "$(dirname "$0")/.."

CLANG_FORMAT="${CLANG_FORMAT:-$(command -v clang-format || echo /opt/homebrew/opt/llvm/bin/clang-format)}"
if [ ! -x "$CLANG_FORMAT" ]; then
  echo "clang-format not found. Install LLVM (brew install llvm) or set CLANG_FORMAT" >&2
  exit 1
fi

if [ "$1" = "--check" ]; then
  find src include tests -name '*.cpp' -o -name '*.hpp' |
    xargs "$CLANG_FORMAT" --dry-run --Werror
else
  find src include tests -name '*.cpp' -o -name '*.hpp' | xargs "$CLANG_FORMAT" -i
fi
