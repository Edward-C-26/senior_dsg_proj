#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#include "main.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

// 12 cells per pack, 6 packs, 4 temps per board
#define NUM_CELLS	72
#define NUM_BOARDS	6
#define NUM_TEMPS	4

typedef struct {
	// General BMS configuration
	uint8_t numOfICs;
	uint8_t address[16];
	uint8_t addressesofICs[16];
	uint8_t numOfCellInputs;
	uint8_t numOfCellsPerIC;
	uint8_t numOfTempPerIC;
	uint8_t ADCConversionRate;
	uint16_t OV_threshold;
	uint16_t UV_threshold;
	uint16_t LUV_threshold;
	uint16_t OT_threshold;
	uint16_t UT_threshold;
	uint16_t HUV_threshold;
	uint16_t slowCharge_threshold;
	uint16_t stopCharge_threshold;
	uint16_t max_difference;
	uint16_t balancing_difference;
	uint16_t balancing_start_threshold;
	uint8_t invalidPECcount;
	uint16_t dischargeTime;
	uint16_t start_scaling;
	uint16_t stop_scaling;
	uint16_t scale_to;
	uint16_t normalCurrent;
	uint16_t lowerCurrent;
	uint16_t chargerVoltage;

	// LTC configuration
	bool GPIO5PulldownOff;
	bool GPIO4PulldownOff;
	bool GPIO3PulldownOff;
	bool GPIO2PulldownOff;
	bool GPIO1PulldownOff;
	bool ReferenceOn;
	bool ADCModeOption;
	uint16_t UndervoltageComparisonVoltage;
	uint16_t OvervoltageComparisonVoltage;
	bool DischargeCell[12];
	uint8_t DischargeTimeoutValue;
	uint8_t ADCMode;
	bool DischargePermitted;

} BMSConfigStructTypedef;

#define CELL_DISCONNECT_MASK (0x001)
#define CELL_PEC_FAIL_MASK (0x01 << 1)
#define CELL_DCFAULT_MASK (0x01 << 3)
#define CELL_TEMP_FAIL_MASK (0x01 << 4)

typedef struct {
    uint8_t fault;
    uint16_t voltage; // LTC6811 ADC measurement - 1 LSB = 100uV = 0.0001V
    uint16_t temperature;
} CellData;

void loadConfig(BMSConfigStructTypedef* config);

#endif	// BMS_CONFIG_H
