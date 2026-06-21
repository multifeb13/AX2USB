#pragma once

#define AX2USB_DEBUG 0
#if AX2USB_DEBUG
#include <Arduino.h>

namespace {

constexpr uint32_t DEBUG_PRINT_DELAY = 500;
Stream& DebugOut = Serial1;

}  // namespace

#define DEBUG_PRINTLN(...)        \
	do {                            \
		DebugOut.printf(__VA_ARGS__); \
		DebugOut.println();           \
	} while (false)
#define DEBUG_PRINTLNw(...)                            \
	do {                                                 \
		static uint32_t last_printed;                      \
		if (millis() - last_printed > DEBUG_PRINT_DELAY) { \
			DebugOut.printf(__VA_ARGS__);                    \
			DebugOut.println();                              \
			last_printed = millis();                         \
		}                                                  \
	} while (false)
#define DEBUG_PRINT(...)          \
	do {                            \
		DebugOut.printf(__VA_ARGS__); \
		DebugOut.println();           \
	} while (false)
#define DEBUG_PRINTw(...)                              \
	do {                                                 \
		static uint32_t last_printed;                      \
		if (millis() - last_printed > DEBUG_PRINT_DELAY) { \
			DebugOut.printf(__VA_ARGS__);                    \
			DebugOut.println();                              \
			last_printed = millis();                         \
		}                                                  \
	} while (false)

#else

#define DEBUG_PRINTLN(fstr, ...) \
	do {                           \
	} while (false)
#define DEBUG_PRINTLNw(fstr, ...) \
	do {                            \
	} while (false)
#define DEBUG_PRINT(fstr, ...) \
	do {                         \
	} while (false)
#define DEBUG_PRINTw(fstr, ...) \
	do {                          \
	} while (false)

#endif
