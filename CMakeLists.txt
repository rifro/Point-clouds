cmake_minimum_required(VERSION 3.18)  # Increased for better CUDA support
# --------------------------------------
# FIX: Force correct linker/stdlib for initial C check
# --------------------------------------
set(CMAKE_C_COMPILER "clang")
set(CMAKE_C_COMPILER_WORKS "TRUE") # Sometimes required if the test is extremely stubborn
set(CMAKE_C_FLAGS "-stdlib=libstdc++") # Set base flag for C
# set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=bfd") # Force BFD linker for the C test. This is likely needed for all executables.
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES "") # Disable C test forcing lld/libc++

project(PluginAutoFit C CXX CUDA)     # Added CUDA to project languages
set(PLUGIN_VERSION "1.0.0")
add_definitions(-DPLUGIN_VERSION="${PLUGIN_VERSION}")

# --------------------------------------
# Compilerinstellingen
# --------------------------------------

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_LINKER "/usr/bin/ld.bfd") # Wellicht overbodig nu
# Configure CUDA compiler compatibility
set(CMAKE_CUDA_COMPILER /usr/bin/nvcc)
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
set(CMAKE_CUDA_STANDARD 17)           # CUDA uses C++17
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------------------
# Paden
# --------------------------------------
set(CC_SRC_ROOT "/workdir/projects/C++/CloudCompare")
set(CC_BUILD_DIR "${CC_SRC_ROOT}/build/debug")

# --------------------------------------
# Qt MOC configuratie
# --------------------------------------
set(Qt5_MOC_EXECUTABLE "/usr/lib/qt5/bin/moc")
if(NOT EXISTS "${Qt5_MOC_EXECUTABLE}")
    message(FATAL_ERROR "moc not found at: ${Qt5_MOC_EXECUTABLE}")
endif()

set(MOC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/moc/moc_autofit.cpp)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/moc)

# MOC generatie commando
add_custom_command(
    OUTPUT ${MOC_OUTPUT}
    COMMAND ${Qt5_MOC_EXECUTABLE}
        -I${CC_SRC_ROOT}/libs/CCPluginStub/include
        -I${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/autofit.h
        -o ${MOC_OUTPUT}
    DEPENDS include/autofit.h
    VERBATIM
    COMMENT "Generating moc file with plugin interface support"
)

# --------------------------------------
# Bestanden verzamelen - INCLUDING CUDA
# --------------------------------------
file(GLOB SOURCE_FILES "src/*.cpp")
file(GLOB HEADER_FILES "include/*.h")
file(GLOB CUDA_SOURCE_FILES "src/*.cu")           # CUDA source files
file(GLOB CUDA_HEADER_FILES "include/*.cuh")      # CUDA header files

# --------------------------------------
# Libraries zoeken
# --------------------------------------
add_library(${PROJECT_NAME} MODULE
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${CUDA_SOURCE_FILES}
    ${CUDA_HEADER_FILES}
    ${MOC_OUTPUT}
)

# Set CUDA architectures (adjust based on your GPU)
set(CMAKE_CUDA_ARCHITECTURES 89) # 4000 series

# --------------------------------------
# Waarschuwingsniveaus - FOR BOTH LANGUAGES
# --------------------------------------
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wno-unused-parameter
    -Wno-old-style-cast
    -Wno-sign-conversion
    -Wno-shadow
    -Wno-deprecated-declarations
    -Wno-conversion
    -Wno-missing-field-initializers
    -Wno-reorder
    -Wno-switch
    -fpermissive
)

# CUDA-specific compile options
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    -Xcompiler=-fpermissive
    -Xptxas=-O3  # Optimaliseer PTX
    -use_fast_math  # Snellere maar minder precieze math
    --expt-relaxed-constexpr
    --extended-lambda
>)

# Expliciet compileren voor debugging met maximale informatie en zonder optimalisaties
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:
  -fno-inline
  -fno-limit-debug-info
  -fno-omit-frame-pointer
  -fsanitize=address
  -fstandalone-debug
  -g
  -gdwarf-5
  -O0
> )

# CUDA-specific debug options
target_compile_options(${PROJECT_NAME} PRIVATE $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CUDA>>:
    -G
    -g
    -O0
>)

# --------------------------------------
# Includes - FOR BOTH C++ AND CUDA
# --------------------------------------
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    include
    ${CC_SRC_ROOT}/libs
    ${CC_SRC_ROOT}/libs/qCC_db/include
    ${CC_SRC_ROOT}/libs/CCPluginAPI/include
    ${CC_SRC_ROOT}/libs/CCPluginStub/include
    ${CC_SRC_ROOT}/libs/qCC_db/extern/CCCoreLib/include
    ${CC_SRC_ROOT}/libs/qCC_glWindow/include
    ${CC_SRC_ROOT}/build/debug/libs/qCC_db/extern/CCCoreLib/exports
    ${CC_SRC_ROOT}/libs/CCCoreLib/include
    ${Qt5_INCLUDE_DIRS} # value not set?
    ${QT5_DIR}
)

# --------------------------------------
# Compile definities - FOR BOTH LANGUAGES
# --------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE 
    QT_PLUGIN
    QCC_PLUGIN_NAME=\"${PROJECT_NAME}\"
    CC_PLUGIN_TYPE_STD
)

# CUDA-specific definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    __CUDA_NO_HALF_CONVERSIONS__
>)

# --------------------------------------
# Target properties
# --------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
    PREFIX "libQ"
    AUTOMOC OFF
    CUDA_RUNTIME_LIBRARY Shared
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
)

# --------------------------------------
# Link met Qt, CC libs EN CUDA
# --------------------------------------
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Find CUDA toolkit
find_package(CUDAToolkit REQUIRED)

# Twee centrale zoekpaden
set(CC_MAIN_LIB_DIR "${CC_SRC_ROOT}/install/release/lib/cloudcompare")
set(CC_PLUGINS_DIR "${CC_MAIN_LIB_DIR}/plugins")

# Directe linking zonder zoekopdrachten - INCLUDING CUDA
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    CUDA::cudart  # Zorgt voor correct geheugenbeheer
    CUDA::cublas
    CUDA::curand
    "${CC_MAIN_LIB_DIR}/libCCCoreLib.so"
    "${CC_MAIN_LIB_DIR}/libQCC_DB_LIB.so"
    "${CC_PLUGINS_DIR}/libQCORE_IO_PLUGIN.so"
)

# --------------------------------------
# Installatie
# --------------------------------------
install(TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION "lib/cloudcompare/plugins"
)

# --------------------------------------
# CUDA Separable Compilation (if needed for complex kernels)
# --------------------------------------
# set_target_properties(${PROJECT_NAME} PROPERTIES
#     CUDA_SEPARABLE_COMPILATION ON
# )
