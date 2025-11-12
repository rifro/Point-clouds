#pragma once

#if defined(__GNUC__) || defined(__clang__)

// Save current warning state
#pragma GCC diagnostic push

// Enable specific warnings as errors
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic error "-Wpedantic"
#pragma GCC diagnostic error "-Wshadow"
#pragma GCC diagnostic error "-Wnon-virtual-dtor"
#pragma GCC diagnostic error "-Wold-style-cast"
#pragma GCC diagnostic error "-Wunused"
#pragma GCC diagnostic error "-Woverloaded-virtual"
#pragma GCC diagnostic error "-Wconversion"
#pragma GCC diagnostic error "-Wsign-conversion"
#pragma GCC diagnostic error "-Wsuggest-override"

#endif
