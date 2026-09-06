// Copyright 2026
// Licensed under Apache‑2.0
#ifndef RAPHAEL_HARDWARE__VISIBILITY_CONTROL_H_
#define RAPHAEL_HARDWARE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"{
#endif

// ==========================================
// Windows (MSVC & MinGW) 处理
// ==========================================
#if defined _WIN32 || defined __CYGWIN__
#ifdef RAPHAEL_HARDWARE_BUILDING_DLL
#ifdef __GNUC__
#define RAPHAEL_HARDWARE_PUBLIC __attribute__((dllexport))
#else
#define RAPHAEL_HARDWARE_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define RAPHAEL_HARDWARE_PUBLIC __attribute__((dllimport))
#else
#define RAPHAEL_HARDWARE_PUBLIC __declspec(dllimport)
#endif
#endif
#define RAPHAEL_HARDWARE_LOCAL
#else
// ==========================================
// Linux / macOS (GCC / Clang) 处理
// ==========================================
// 优先 __has_attribute，兼容Clang；再回退GCC版本判断
#if defined(__has_attribute) && __has_attribute(visibility)
#define RAPHAEL_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define RAPHAEL_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && __GNUC__ >= 4
#define RAPHAEL_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define RAPHAEL_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
#else
// 降级处理：不支持 visibility 的编译器
#define RAPHAEL_HARDWARE_PUBLIC
#define RAPHAEL_HARDWARE_LOCAL
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // RAPHAEL_HARDWARE__VISIBILITY_CONTROL_H_
