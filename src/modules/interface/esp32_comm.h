#pragma once

#include <stdint.h>
#include <stdbool.h>

// Call this once at startup
void esp32CommInit(void);

// Call this from your radio handling code to switch controllers
void esp32CommSetControllerId(uint16_t id);

// Call this from your 1000Hz stabilizer loop
// Returns a pointer to the 28 floats, or NULL if no valid controller is loaded yet
const float* esp32CommGetActiveController(void);