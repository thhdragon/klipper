#ifndef KLIPPER_RUST_PORT_H
#define KLIPPER_RUST_PORT_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Implement the standard crc "ccitt" algorithm on the given buffer
 */
uint16_t crc16_ccitt(const uint8_t *buf, unsigned char len);

#endif  /* KLIPPER_RUST_PORT_H */
