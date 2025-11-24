#!/usr/bin/env python3
import os
import subprocess
import sys

PROJECT_NAME = "PluginAutoFit"
BUILD_DIR = "build"
PLUGIN_DIR = "plugins"

os.system(f"rm -rf {BUILD_DIR} {PLUGIN_DIR}")
os.makedirs(BUILD_DIR, exist_ok=True)
os.makedirs(PLUGIN_DIR, exist_ok=True)

CC_SRC_ROOT = "/workdir/projects/C++/CloudCompare"
CC_MAIN_LIB_DIR = f"{CC_SRC_ROOT}/install/release/lib/cloudcompare"
CC_PLUGINS_DIR = f"{CC_MAIN_LIB_DIR}/plugins"
QT5_DIR = "/usr/lib/qt5"

C_FLAGS = "-Wall -Wno-unused-parameter -Wno-old-style-cast -Wno-sign-conversion -Wno-shadow -Wno-deprecated-declarations -Wno-conversion -Wno-missing-field-initializers -Wno-reorder -Wno-switch -fpermissive -stdlib=libstdc++"
CXX_FLAGS = f"{C_FLAGS} -std=c++20 -g -O0"
CUDA_FLAGS = "-std=c++17 --compiler-options='-fpermissive -stdlib=libstdc++' -Xptxas=-O3 -use_fast_math --expt-relaxed-constexpr --extended-lambda -arch=sm_89 -g -O0"

# MOC Qt
MOC = f"{QT5_DIR}/bin/moc"
MOC_SRC = f"{BUILD_DIR}/moc_autofit.cpp"
MOC_OBJ = f"{BUILD_DIR}/moc_autofit.o"

# 1. Generate MOC source file (FIXED: correct f-string syntax)
subprocess.check_call([MOC, "-I./include", f"-I{CC_SRC_ROOT}/libs/CCPluginStub/include", "include/autofit.h", "-o", MOC_SRC])
print(f"MOC generated: {MOC_SRC}")

# Include directories list
INCLUDES = [
    "-I./include",
    f"-I{CC_SRC_ROOT}/libs",
    f"-I{CC_SRC_ROOT}/libs/qCC_db/include",
    f"-I{CC_SRC_ROOT}/libs/CCPluginAPI/include",
    f"-I{CC_SRC_ROOT}/libs/CCPluginStub/include",
    f"-I{CC_SRC_ROOT}/libs/qCC_db/extern/CCCoreLib/include",
    f"-I{CC_SRC_ROOT}/libs/qCC_glWindow/include",
    f"-I{CC_SRC_ROOT}/build/debug/libs/qCC_db/extern/CCCoreLib/exports",
    f"-I{CC_SRC_ROOT}/libs/CCCoreLib/include",
    f"-I{QT5_DIR}/include"
]

# 2. Compile MOC source file separately
subprocess.check_call(["clang++", "-c", CXX_FLAGS] + INCLUDES + [MOC_SRC, "-o", MOC_OBJ])
print(f"MOC compiled: {MOC_OBJ}")
cxx_objects = [MOC_OBJ]

# 3. Compile CXX
for src in os.listdir("src"):
    if src.endswith(".cpp"):
        base = src[:-4]
        obj = f"{BUILD_DIR}/{base}.o"
        # FIX: Removed MOC_SRC from the input list.
        subprocess.check_call(["clang++", "-c", CXX_FLAGS] + INCLUDES + [f"src/{src}", "-o", obj])
        cxx_objects.append(obj)
print("CXX sources compiled.")

# 4. Compile CUDA (if .cu)
cuda_objects = []
CUDA_INCLUDES = [
    "-I./include", 
    f"-I{CC_SRC_ROOT}/libs", 
    f"-I{QT5_DIR}/include"
]

for cu in os.listdir("src"):
    if cu.endswith(".cu"):
        base = cu[:-3]
        obj = f"{BUILD_DIR}/{base}.o"
        # FIX: Correct f-string syntax for paths
        subprocess.check_call(["nvcc", "-c", CUDA_FLAGS] + CUDA_INCLUDES + [f"src/{cu}", "-o", obj])
        cuda_objects.append(obj)
print("CUDA sources compiled.")

# 5. Link plugin
objects = cxx_objects + cuda_objects
LINK_LIBS = [
    f"-L{CC_MAIN_LIB_DIR}", "-lCCCoreLib", "-lQCC_DB_LIB", 
    f"-L{CC_PLUGINS_DIR}", "-lQCORE_IO_PLUGIN", 
    f"-L{QT5_DIR}/lib", "-lQt5Core", "-lQt5Gui", "-lQt5Widgets", 
    "-lcudart", "-lcublas", "-lcurand", 
    "-fuse-ld=bfd", "-stdlib=libstdc++", "-g", "-O0"
]
subprocess.check_call(["clang++", "-shared", "-o", f"{PLUGIN_DIR}/libQ{PROJECT_NAME}.so"] + objects + LINK_LIBS)

print(f"Built libQ{PROJECT_NAME}.so in {PLUGIN_DIR}.")
