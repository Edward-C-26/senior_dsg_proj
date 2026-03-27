#include "BMSconfig.h"

void loadConfig(BMSConfigStructTypedef* cfg) {
    cfg->numOfICs = 1;
    cfg->address[0] = 0;

    cfg->numOfCellInputs = 6;
    cfg->numOfCellsPerIC = 6;
    cfg->numOfTempPerIC = 2;

    // Fault thresholds (100 uV units for voltage, mC for temperature)
    cfg->OV_threshold = 42000;      // 4.20 V
    cfg->UV_threshold = 30000;      // 3.00 V
    cfg->OT_threshold = 60000;      // 60 C
    cfg->UT_threshold = 0;          // 0 C

    // Invalid-reading / warning thresholds
    cfg->LUV_threshold = 25000;
    cfg->HUV_threshold = 45000;

    // Charge-related fields: unused in discharge-only system
    cfg->slowCharge_threshold = 0;
    cfg->stopCharge_threshold = 0;
    cfg->normalCurrent = 0;
    cfg->lowerCurrent = 0;
    cfg->chargerVoltage = 0;

    // Imbalance / discharge decision thresholds
    // These are for deciding isolation or optional balancing/demo behavior
    cfg->balancing_start_threshold = 41000;   // optional demo threshold
    cfg->balancing_difference = 100;          // 10 mV
    cfg->max_difference = 200;                // 20 mV imbalance limit
    cfg->start_scaling = 0;
    cfg->stop_scaling = 0;
    cfg->scale_to = 0;

    cfg->invalidPECcount = 5;
    cfg->dischargeTime = 500;

    cfg->ADCConversionRate = 0;
    cfg->ADCModeOption = 0;

    cfg->GPIO5PulldownOff = 1;
    cfg->GPIO4PulldownOff = 1;
    cfg->GPIO3PulldownOff = 1;
    cfg->GPIO2PulldownOff = 1;
    cfg->GPIO1PulldownOff = 1;

    cfg->ReferenceOn = 1;

    cfg->UndervoltageComparisonVoltage = 0x000;
    cfg->OvervoltageComparisonVoltage = 0x000;

    for (uint8_t i = 0; i < 12; i++) {
        cfg->DischargeCell[i] = 0;
    }

    cfg->DischargeTimeoutValue = 0x0;
}