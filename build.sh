#!/bin/bash
set -e  # Exit on error
PROJECT_NAME="PluginAutoFit"
BUILD_DIR="build"
PLUGIN_DIR="plugins"

# Clean
rm -rf $BUILD_DIR $PLUGIN_DIR
mkdir -p $BUILD_DIR $PLUGIN_DIR

# Paths (your from CMake)
CC_SRC_ROOT="/workdir/projects/C++/CloudCompare"
CC_MAIN_LIB_DIR="${CC_SRC_ROOT}/install/release/lib/cloudcompare"
CC_PLUGINS_DIR="${CC_MAIN_LIB_DIR}/plugins"
#QT5_DIR="/usr/lib/qt5"  # Already set

# Flags (your original + libstdc++ for ABI)
C_FLAGS="-Wall -Wno-unused-parameter -Wno-old-style-cast -Wno-sign-conversion -Wno-shadow -Wno-deprecated-declarations -Wno-conversion -Wno-missing-field-initializers -Wno-reorder -Wno-switch -fpermissive -stdlib=libstdc++"
CXX_FLAGS="$C_FLAGS -std=c++20 -stdlib=libstdc++ -g -O0"  # Debug
CUDA_FLAGS="-std=c++17 --compiler-options='-fpermissive -stdlib=libstdc++' -Xptxas=-O3 -use_fast_math --expt-relaxed-constexpr --extended-lambda -arch=sm_89 -g -O0"  # 4000 series debug

# MOC Qt
MOC="$QT5_DIR/bin/moc"
MOC_SRC="$BUILD_DIR/moc_autofit.cpp"
MOC_OBJ="$BUILD_DIR/moc_autofit.o"

if [ ! -f "$MOC" ]; then
    echo "Error: moc not found at $MOC"
    exit 1
fi

# 1. Generate MOC source file
$MOC -I./include -I"$CC_SRC_ROOT/libs/CCPluginStub/include" include/autofit.h -o $MOC_SRC
echo "MOC generated: $MOC_SRC"

# 2. Compile MOC source file separately
# This prevents the "cannot specify -o when generating multiple output files" error
INCLUDES="-I./include -I$CC_SRC_ROOT/libs -I$CC_SRC_ROOT/libs/qCC_db/include -I$CC_SRC_ROOT/libs/CCPluginAPI/include -I$CC_SRC_ROOT/libs/CCPluginStub/include -I$CC_SRC_ROOT/libs/qCC_db/extern/CCCoreLib/include -I$CC_SRC_ROOT/libs/qCC_glWindow/include -I$CC_SRC_ROOT/build/debug/libs/qCC_db/extern/CCCoreLib/exports -I$CC_SRC_ROOT/libs/CCCoreLib/include -I$QT5_DIR/include"
clang++ -c $CXX_FLAGS $INCLUDES "$MOC_SRC" -o $MOC_OBJ
echo "MOC compiled: $MOC_OBJ"

# Start object list with the MOC object
OBJECTS="$MOC_OBJ"

# 3. Compile CXX sources (.cpp)
for src in src/*.cpp; do
    base=$(basename "$src" .cpp)
    OBJ="$BUILD_DIR/${base}.o"
    # Note: $MOC_SRC is NOT included here, only one input file ($src) is used with -o
    clang++ -c $CXX_FLAGS $INCLUDES "$src" -o $OBJ
    OBJECTS="$OBJECTS $OBJ"
done
echo "CXX sources compiled."

# 4. Compile CUDA sources (.cu) – optional if no .cu
if ls src/*.cu 1> /dev/null 2>&1; then
    for cu in src/*.cu; do
        base=$(basename "$cu" .cu)
        OBJ="$BUILD_DIR/${base}.o"
        # Note: CUDA include paths are slightly different
        nvcc -c $CUDA_FLAGS -I./include -I"$CC_SRC_ROOT/libs" -I"$QT5_DIR/include" "$cu" -o $OBJ
        OBJECTS="$OBJECTS $OBJ"
    done
    echo "CUDA sources compiled."
else
    echo "No .cu files – skipping CUDA compile."
fi

# 5. Link plugin
clang++ -shared -o $PLUGIN_DIR/libQ$PROJECT_NAME.so $OBJECTS -L"$CC_MAIN_LIB_DIR" -lCCCoreLib -lQCC_DB_LIB -L"$CC_PLUGINS_DIR" -lQCORE_IO_PLUGIN -L"$QT5_DIR/lib" -lQt5Core -lQt5Gui -lQt5Widgets -lcudart -lcublas -lcurand -fuse-ld=bfd -stdlib=libstdc++ -g -O0
echo "Linked: $PLUGIN_DIR/libQ$PROJECT_NAME.so"
ls -l $PLUGIN_DIR/libQ$PROJECT_NAME.so
