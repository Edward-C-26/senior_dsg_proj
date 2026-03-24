#ifndef LTC6811_NO_MALLOCS_H
#define LTC6811_NO_MALLOCS_H

#include "BMSconfig.h"
#include "SPI.h"
#include "math.h"
// Lock your computer bro --Louis Ye

// ADC measurement to voltage conversion constant
#define LTC6811_ADC_LSB_PER_V 10000

static const float lookupTableTemps[] = {
	//203 value lookup table to avoid Stenhart-Hart equation.
	//Resolution increases around 40c to account for change in equation
	//Values are in 0.5c increments
	70, 68.5, 67, 66, 64.5, 63, 62, 61, 60, 59, 58, 57, 56, 55.5, 55, 54, 53, 52, 51.5, 51, 50, 49.5, 49, 48, 47.5, 47, 46, 45.5, 45, 44.5, 44, 43.5, 43, 42, 41.5, 41, 40.5,
	40, 40, 39.5, 39.0, 38.5, 38.0, 37.5, 37.0, 36.5, 36.0, 36, 35.5, 35.0, 34.5, 34.0, 34, 33.5, 33.0, 32.5, 32, 32.0, 31.5, 31, 31.0, 30.5, 30.0, 30, 29.5, 29.0, 29, 28.5, 
	28, 28.0, 27.5, 27, 27.0, 26.5, 26, 26.0, 26, 25.5, 25, 25.0, 24.5, 24, 24.0, 24, 23.5, 23, 23.0, 23, 22.5, 22, 22.0, 22, 21.5, 21, 21.0, 21, 20.5, 20, 20.0, 20, 19.5, 19, 
	19.0, 19, 18.5, 18, 18.0, 18, 17.5, 17, 17.0, 17, 17, 16.5, 16, 16.0, 16, 15.5, 15, 15, 15.0, 15, 14.5, 14, 14.0, 14, 14, 13.5, 13, 13.0, 13, 13, 12.5, 12, 12.0, 12, 12, 
	11.5, 11, 11.0, 11, 11, 10.5, 10, 10, 10.0, 10, 9.5, 9, 9, 9.0, 9, 9, 8.5, 8, 8.0, 8, 8, 7.5, 7, 7, 7.0, 7, 7, 6.5, 6, 6, 6.0, 6, 6, 5.5, 5, 5, 5.0, 5, 4.5, 4, 4, 4.0, 4, 
	4, 3.5, 3, 3, 3.0, 3, 3, 2.5, 2, 2, 2, 2.0, 2, 2, 1.5, 1, 1, 1.0, 1, 1, 0.5, 0, 0, 0.0
	
	
	};




typedef enum {
	WriteConfigurationRegisterGroup = 0x001,
	ReadConfigurationRegisterGroup = 0x002,
	ReadCellVoltageRegisterGroup1to3 = 0x004,
	ReadCellVoltageRegisterGroup4to6 = 0x006,
	ReadCellVoltageRegisterGroup7to9 = 0x008,
	ReadCellVoltageRegisterGroup10to12 = 0x00A,
	ReadAuxiliaryGroupA = 0x00C,
	ReadAuxiliaryGroupB = 0x00E,
	StartOpenWireConversionPulldown = 0x229,
//	StartCellVoltageADCConversionAll = 0x260,	   // MD = 00, DCP = 0, CHG = 000
	StartCellVoltageADCConversionAll = 0x360,
//	StartCellTempVoltageADCConversionAll = 0x460,  // MD = 00, CHG = 000
	StartCellTempVoltageADCConversionAll = 0x560,  // MD = 00, CHG = 000
	ClearRegisters = 0x711
} CommandCodeTypedef;

void initPECTable(void);
void writeConfigAddress(BMSConfigStructTypedef *cfg, uint8_t address);
void writeConfigAll(BMSConfigStructTypedef *cfg);
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]);


bool readAllCellVoltages(CellData bmsData[]);
bool readCellTemp(uint8_t address, uint16_t cellTemp[4], bool dcFault[4], bool tempFault[4]);
bool readAllCellTemps(CellData bmsData[]);
// bool checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]);
bool checkAllCellConnections(BMSConfigStructTypedef cfg, CellData bmsData[]);
bool dischargeCellGroups(BMSConfigStructTypedef *cfg, bool cellDischarge[12][12]);
void wakeup_idle();
// bool dischargeCell(BMSConfigStructTypedef config, bool cellDischarge[8]);
bool readConfig(uint8_t address, uint8_t cfg[8]);
bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data);
void sendBroadcastCommand(CommandCodeTypedef command);
void sendAddressCommand(CommandCodeTypedef command, uint8_t address);
uint16_t calculatePEC(uint8_t len, uint8_t *data);
bool poll_single_secondary_voltage_reading(uint8_t board_num, BMSConfigStructTypedef *cfg, CellData bmsData[]);
bool poll_single_secondary_temp_reading(uint8_t board_num, BMSConfigStructTypedef *cfg, CellData bmsData[]);

#endif	// LTC6811_NO_MALLOCS_H
