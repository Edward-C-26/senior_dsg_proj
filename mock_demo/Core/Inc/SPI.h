#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

bool SPIWrite(uint8_t *writeBuffer, uint8_t totalBytes);
bool SPIWriteRead(uint8_t *writeBuffer, uint8_t *readBuffer, uint8_t totalBytes);

#ifdef __cplusplus
}
#endif

#endif
