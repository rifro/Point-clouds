cmake_minimum_required(VERSION 3.16)
project(PluginAutoFit C CXX)

# --------------------------------------
# Compilerinstellingen
# --------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Waarschuwingsniveaus
add_compile_options(
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
# Bestanden verzamelen
# --------------------------------------
file(GLOB SOURCE_FILES "src/*.cpp")
file(GLOB HEADER_FILES "include/*.h")

# --------------------------------------
# Libraries zoeken
# --------------------------------------
add_library(${PROJECT_NAME} MODULE
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${MOC_OUTPUT}
)

# Expliciet compileren voor debugging met maximale informatie en zonder optimalisaties
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:
  -fno-inline
  -fno-limit-debug-info
  -fno-omit-frame-pointer
  -fstandalone-debug
  -g
  -gdwarf-5
  -O0
> )

# --------------------------------------
# Includes
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
    ${Qt5_INCLUDE_DIRS}
)

# --------------------------------------
# Compile definities
# --------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE 
    QT_PLUGIN
    QCC_PLUGIN_NAME=\"${PROJECT_NAME}\"
    CC_PLUGIN_TYPE_STD
)

# --------------------------------------
# Target properties
# --------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
    PREFIX "libQ"
    AUTOMOC OFF
)

# --------------------------------------
# Link met Qt en CC libs
# --------------------------------------
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Twee centrale zoekpaden
set(CC_MAIN_LIB_DIR "${CC_SRC_ROOT}/install/release/lib/cloudcompare")
set(CC_PLUGINS_DIR "${CC_MAIN_LIB_DIR}/plugins")

# Directe linking zonder zoekopdrachten
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
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
