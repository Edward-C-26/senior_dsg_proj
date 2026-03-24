

#include "BMSconfig.h"

void loadConfig(BMSConfigStructTypedef* cfg) {
	// change this
	cfg->numOfICs = 12;
	for (uint8_t i = 0; i < 12; i++) {
		cfg->address[i] = i;
	}
	// cfg->address[0] = 0;
	// cfg->address[1] = 1;

	cfg->numOfCellInputs = 12;
	cfg->numOfCellsPerIC = 12;
	cfg->numOfTempPerIC = 4;

	// OV_threshold and UV_threshold: used for OV and UV fault-checking

	cfg->OV_threshold = 41600;	// maximum on cell spec sheet is 4.2V, but 600/144 = 4.167
	cfg->UV_threshold = 32000;

	cfg->OT_threshold = 50*1000;		// comp is 60
	cfg->UT_threshold = 17*1000;		// comp is 20

	cfg->LUV_threshold = 22500;
	cfg->HUV_threshold = 27500;
	// charge to 4.14V, lower charge current, discharge cells above 4.16V
	// exceed 4.18V, stop charging entirely, discharge to 4.15V
	// exceed 4.2V, fault prevents discharging

	// cell voltage limit 4.16V
	// full current or no current
	// exceed 4.17 on any cell, stop charging, discharge that one cell to 4.15
	// start charging again

	// if there is a big enough delta above minimum, stop charging so that you can actually lower voltage

	// UV FAULT 2.5, check ESF
	// if cells are below 3, don't balance

	cfg->slowCharge_threshold = 32700;	// start charging slowly at total V = 471.
										// 471/144 = 3.27 V percell
	cfg->stopCharge_threshold = 41500;	// maximum on cell spec sheet is 4.2V, but we want to ensure that the total voltage does not exceed 600V.
										// 600/144 = 4.167, use 4.15 to be safe

	// START FROM HERE NEXT TIME --------------------------------------------------------------------------------------
	cfg->max_difference = 2000;
	cfg->balancing_difference = 500;
	cfg->balancing_start_threshold = 40000;		// start discharging accumulator at 4V for max cell
	cfg->start_scaling = 41000;	 // 41000;
	cfg->stop_scaling = 41600;	 // 41600;
	cfg->scale_to = 100;

	cfg->invalidPECcount = 5;

	cfg->dischargeTime = 1500;  // ms

	// cfg->normalCurrent = 0xA;
	// cfg->lowerCurrent = 0x5;
	// cfg->chargerVoltage = 0x1770;

	// ESPL
	// cfg->normalCurrent = 0x06E;   // 11.0A
	// cfg->lowerCurrent = 0x000A;    // 1.0A
	// cfg->chargerVoltage = 0x1770;  // 600.0V

	// FSAE Comp 208v
	cfg->normalCurrent = 0x0037;   // 5.5A          0x14; // 2.0 A, used for accumulator inspection
	cfg->lowerCurrent = 0x000A;	   // 1.0A
	cfg->chargerVoltage = 0x1770;  // 600.0V

	// FSAE Comp 240v
	// cfg->normalCurrent = 0x0040;   // 6.4A
	// cfg->lowerCurrent = 0x000A;    // 1.0A
	// cfg->chargerVoltage = 0x1770;  // 600.0V


	// this is used to set the adc rate in the ADCV command
	cfg->ADCConversionRate = 0;
	// this sets ADCOPT(CFGR0[0]), see pg 61 for more details
	cfg->ADCModeOption = 0; // How TF is this calculated?????

	// chip and code should share OV and UV thresholds

	// cfg->ADCModeOption = (config.ADCConversionRate) & 0b011;
	// cfg->ADCMode = (config.ADCConversionRate) & 0b100;

	cfg->GPIO5PulldownOff = 1;
	cfg->GPIO4PulldownOff = 1;
	cfg->GPIO3PulldownOff = 1;
	cfg->GPIO2PulldownOff = 1;
	cfg->GPIO1PulldownOff = 1;

	cfg->ReferenceOn = 1;  // minimizes time between conversions

	cfg->UndervoltageComparisonVoltage = 0x000;
	cfg->OvervoltageComparisonVoltage = 0x000;

	for (uint8_t i = 0; i < 12; i++)
		cfg->DischargeCell[i] = 0;

	cfg->DischargeTimeoutValue = 0x0;
}
