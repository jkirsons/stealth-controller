#ifndef CANAPI_H
#define CANAPI_H
#include <stdint.h>

void _initCAN(int tx, int rx);
void _transmitCAN(uint32_t identifier, uint8_t *data, uint8_t length);
bool _receiveCAN(uint32_t *identifier, uint8_t *data, uint8_t *length);
void _getUniqueID(uint8_t * id);

#endif