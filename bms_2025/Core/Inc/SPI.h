#ifndef SPI_H
#define SPI_H

#include "main.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

#define MAX_SPI_DELAY 3

void SPI_Init(void);
bool SPIWrite(uint8_t *writeBuffer, uint8_t totalBytes);
bool SPIWriteRead(uint8_t *writeBuffer, uint8_t *readBuffer, uint8_t totalBytes);

#endif	// SPI_H
