#!/usr/bin/env bash
# post_cubemx.sh — Run this after every STM32CubeMX code regeneration.
#
# CubeMX overwrites cmake/stm32cubemx/CMakeLists.txt and re-adds the X-CUBE-AI
# sources, include paths, and runtime library into that file.  Because the main
# CMakeLists.txt already owns all of that configuration (so it survives regen),
# having the entries in both files causes duplicate-symbol link errors.
# This script removes the AI-specific lines from the CubeMX-managed file.

set -e
FILE="cmake/stm32cubemx/CMakeLists.txt"

if [[ ! -f "$FILE" ]]; then
  echo "ERROR: $FILE not found. Run from the project root." >&2
  exit 1
fi

echo "Patching $FILE ..."

# Remove X-CUBE-AI include directories
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./Middlewares/ST/AI/Inc|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/AI/Inc  # owned by main CMakeLists.txt|' "$FILE"
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./X-CUBE-AI/App$|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../X-CUBE-AI/App  # owned by main CMakeLists.txt|' "$FILE"

# Remove X-CUBE-AI generated network sources
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./X-CUBE-AI/App/network_data\.c|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../X-CUBE-AI/App/network_data.c  # owned by main CMakeLists.txt|' "$FILE"
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./X-CUBE-AI/App/network\.c|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../X-CUBE-AI/App/network.c  # owned by main CMakeLists.txt|' "$FILE"
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./X-CUBE-AI/App/network_data_params\.c|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../X-CUBE-AI/App/network_data_params.c  # owned by main CMakeLists.txt|' "$FILE"

# Remove AI lib search directory
sed -i 's|    \${CMAKE_CURRENT_SOURCE_DIR}/\.\./\.\./Middlewares/ST/AI/Lib|    # ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/AI/Lib  # owned by main CMakeLists.txt|' "$FILE"

# Remove runtime library entry (match any leading whitespace)
sed -i 's|[[:space:]]*:NetworkRuntime1020_CM4_GCC\.a|    # :NetworkRuntime1020_CM4_GCC.a  # owned by main CMakeLists.txt|' "$FILE"

echo "Done. Verify with:"
echo "  grep -n 'X-CUBE-AI\|NetworkRuntime\|Middlewares/ST/AI' $FILE"
