// log.h -- centralized logging framework (v16.6 logging refactor)
//
// Replaces scattered Serial.print()/Serial.printf() diagnostics with
// level-gated macros. Changing LOG_LEVEL below is the single control point
// for what a build emits; disabled levels compile out completely (the
// macro expands to nothing), so there is zero runtime cost either way.
//
// Levels are plain #define integers, not an enum -- this sketch's .ino
// auto-prototype convention requires custom enum/struct types used as a
// function return/parameter to be typedef'd ahead of the first function in
// the file (see CLAUDE.md). Plain integer levels sidestep that entirely,
// since LOG_LEVEL is only ever compared at preprocessor/compile time, never
// passed as a function argument.
#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

#define LOG_NONE  0
#define LOG_ERROR 1
#define LOG_WARN  2
#define LOG_INFO  3
#define LOG_DEBUG 4
#define LOG_TRACE 5

// Single line controlling all logging in the sketch.
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

// Common emit step shared by every level macro below, so the
// "Serial.printf(prefix fmt, ...)" formatting logic exists in exactly one
// place. Each level still needs its own #if/#else pair (not folded into
// this helper) because each level's enabled/disabled state depends on a
// different LOG_LEVEL comparison -- that per-level gating is what makes
// disabled levels compile out completely.
#define LOG_EMIT(prefix, fmt, ...) Serial.printf(prefix fmt, ##__VA_ARGS__)

#if LOG_LEVEL >= LOG_ERROR
#define LOGE(fmt, ...) LOG_EMIT("[ERROR] ", fmt, ##__VA_ARGS__)
#else
#define LOGE(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_WARN
#define LOGW(fmt, ...) LOG_EMIT("[WARN ] ", fmt, ##__VA_ARGS__)
#else
#define LOGW(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_INFO
#define LOGI(fmt, ...) LOG_EMIT("[INFO ] ", fmt, ##__VA_ARGS__)
#else
#define LOGI(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_DEBUG
#define LOGD(fmt, ...) LOG_EMIT("[DEBUG] ", fmt, ##__VA_ARGS__)
#else
#define LOGD(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_TRACE
#define LOGT(fmt, ...) LOG_EMIT("[TRACE] ", fmt, ##__VA_ARGS__)
#else
#define LOGT(fmt, ...) ((void)0)
#endif

#endif  // LOG_H
