#pragma once
#include <cstdint>
enum peripheral_bus_type_t { ESP32_BUS_TYPE_INIT, ESP32_BUS_TYPE_I2C_MASTER_SDA };
inline peripheral_bus_type_t perimanGetPinBusType(uint8_t) { return ESP32_BUS_TYPE_INIT; }
