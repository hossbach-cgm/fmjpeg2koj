#ifndef FMJPEG2K_CXX_CONFIG_H
#define FMJPEG2K_CXX_CONFIG_H

// Check the consumer's actual language mode. Do not enable it implicitly via
// transitive compile features: the consuming project must select C++20 or newer.
// MSVC reports the selected mode in _MSVC_LANG even without /Zc:__cplusplus.
#if defined(_MSVC_LANG)
#  if _MSVC_LANG < 202002L
#    error "fmjpeg2k requires C++20 or newer; enable /std:c++20 (MSVC) or -std=c++20."
#  endif
#elif !defined(__cplusplus) || __cplusplus < 202002L
#  error "fmjpeg2k requires C++20 or newer; enable /std:c++20 (MSVC) or -std=c++20."
#endif

#endif
