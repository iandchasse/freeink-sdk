#pragma once
// BatteryMonitor.cpp logs through the app's Logging.h, which the SDK does not carry.
// With ENABLE_SERIAL_LOG off, the real macros expand to nothing as well.
#define LOG_DBG(origin, format, ...)
#define LOG_ERR(origin, format, ...)
#define LOG_INF(origin, format, ...)
