#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "PackCalculations.h"
#include "BMSconfig.h"
#include "Fault.h"

#include <assert.h>

// Do not change
#define V_SCALE 10000
// See where the #defines are being used,
// you can replace them to change behaviour
#define CHARGE_RATE 4
#define PRINT_STEP 1000
#define NEXT_CELL_VOLT_LOGS 0
#define LOGS 0
#define MAX_STEP_COUNT 10000
#define TIME_STEP_MS (double)(1000*60)
#define VOLT_NOM 37000
#define VOLT_MINIMUM 30000
#define VOLT_MAXIMUM 44000
#define VOLT_BALANCE 41300
#define BALANCED_VOLT_DIFF 1000
#define VOLT_DEV 3000

void printBMSAsInt(BMSConfigStructTypedef const *cfg,
        CellData const bmsData[144],
        double chargeRate) {
    printf("Accumulator State | Charging at %f\n", chargeRate);
	for (uint8_t i = 0; i < 12; i++) {
		for (uint8_t j = 0; j < 12; j++) {
            printf("%d ", bmsData[i*12+j].voltage);
        }
        printf("\n");
    }
        printf("\n\n");
}

void printBMS(BMSConfigStructTypedef *cfg,
        CellData bmsData[144], 
        double chargeRate) {
    printf("Accumulator State | Charging at %f\n", chargeRate);
	for (uint8_t i = 0; i < cfg->numOfICs; i++) {
		for (uint8_t j = 0; j < 12; j++) {
            printf("%.4f ", ((double)bmsData[i*12+j].voltage)/V_SCALE);
        }
        printf("\n");
    }
    printf("\n\n");
}

double calculateDischargeCurrent(double voltage) {
    // KVL in discharge circuit
    double ret = 0.12*voltage - 0.052;
#if LOG == 1
    printf("current %f\n", ret);
#endif
    return ret;
}

double voltToAmpacitance(double voltFloat) {
    double ret = 
            -7.6805718154486080e+004+
            1.0795195796487253e+005 * voltFloat+
            -6.0397660207008295e+004 * pow(voltFloat, 2)+
            1.6812490151230424e+004* pow(voltFloat, 3)+
            -2.3284497782538242e+003* pow(voltFloat, 4)+
            1.2837460057248754e+002* pow(voltFloat, 5);
#if LOGS == 1
    printf("Ampacitance %f\n", ret);
#endif
    return ret;
}

double deltaVDeltaAmpacity(double ampacity) {
    double ret =(
    -2.4*(pow(10,-5))*(pow(ampacity, 5)) +
    0.001*(pow(ampacity, 4)) +
    -0.0136*(pow(ampacity,3)) +
    +0.0980999888*(pow(ampacity,2)) +
    -0.321*(ampacity) +
    0.4015
    );
#if LOGS==1
    printf("dv/dA %f\n", ret);
#endif
    return ret;
}

// Due to nature of polynomial functions, if the voltage somehow reaches
// anything that's not safe, everything gets fucked up
uint16_t nextCellVoltage(uint16_t voltage, double current, double timeMs) {

    double deltaAmpacity = current * (timeMs / (1000*60*60));
    double voltFloat = ((double)voltage / V_SCALE);
    double ampacity = voltToAmpacitance(voltFloat);
    uint16_t  ret =   voltage + 
        ((uint16_t)(deltaVDeltaAmpacity(ampacity) * deltaAmpacity * V_SCALE));
#if LOGS==1 || NEXT_CELL_VOLT_LOGS == 1
    printf("voltageIn %f\n", (float)(voltage)/V_SCALE);
    printf("voltageDif %f\n", (float)(ret - voltage)/V_SCALE);
#endif
    return ret;
}

void tickBattery(BMSConfigStructTypedef *cfg,
                 CellData bmsData[144],
                 bool cellDischarge[12][12],
                 uint8_t chargeRate
                 ) {
    for (uint8_t i = 0; i < cfg->numOfICs; i++) {
        for (uint8_t j = 0; j < 12; j++) {
            uint16_t voltage =  bmsData[i*12+j].voltage;
            double voltFloat = ((double)voltage)/V_SCALE;
            double netCurrent = (double) chargeRate;
#if LOGS==1
            printf("Curr Cell %d\n", i*12 + j);
#endif
            if (cellDischarge[i][j]) {
                double dischargeCurrent = calculateDischargeCurrent(voltFloat);
                netCurrent -= dischargeCurrent;
            }

            bmsData[i*12+j].voltage = 
                nextCellVoltage(voltage, netCurrent, TIME_STEP_MS);
#if LOGS==1
            printf("\n");
#endif
        }

    }
}

void setBatteryVoltagesNom(CellData bmsData[144], 
        bool cellDischarge[12][12], bool fullDischarge[12][12]){
    for (int cell = 0; cell < 144; cell++) {
        bmsData[cell].voltage = VOLT_NOM;
        cellDischarge[cell / 12][cell % 12] = 0;
        fullDischarge[cell / 12][cell % 12] = 0;
    }
}

void setBatteryVoltagesImbalanced(CellData bmsData[144],
        bool cellDischarge[12][12], bool fullDischarge[12][12]){
    for (int cell = 0; cell < 144; cell++) {
        int16_t dev = ((int16_t)rand() % VOLT_DEV) - VOLT_DEV/2;
        bmsData[cell].voltage = (uint16_t)(VOLT_NOM + dev);
        cellDischarge[cell / 12][cell % 12] = 0;
        fullDischarge[cell / 12][cell % 12] = 0;
    }
}

bool eqWithTolUI(uint16_t first, uint16_t second, uint16_t tol) {
    return (first < second+tol && first > second-tol);
}

bool eqWithTolF(double first, double second, double tol) {
    return (first < second+tol && first > second-tol);
}

void testCellBalance() {
    BMSConfigStructTypedef config;
    BMS_critical_info_t criticalInfo;
    loadConfig(&config);
    init_BMS_info(&criticalInfo);
    bool cellDischarge[12][12];
    bool fullDischarge[12][12];
    uint8_t balanceCounter = 0;
    uint8_t chargeRate = CHARGE_RATE;

    CellData bmsData[144];
    setBatteryVoltagesImbalanced(bmsData, cellDischarge, fullDischarge);
    // Replace some voltages with balance initiating voltage.

    printBMS(&config, bmsData, chargeRate);
    for (int i = 0; i < MAX_STEP_COUNT; i++ ) {
        // BMS internal state update functions
        setCriticalVoltages(&criticalInfo, bmsData);
        balance(&config, &criticalInfo, bmsData, cellDischarge,
                fullDischarge, balanceCounter, &chargeRate);
        balanceCounter++;
        if (chargeRate != 0) {
            tickBattery(&config, bmsData, cellDischarge, chargeRate);
        } else {
            tickBattery(&config, bmsData, fullDischarge, chargeRate);
        }

        // Print current state
        if (i % PRINT_STEP == 0) {
            printBMS(&config, bmsData, chargeRate);
        }

        // Test checks
        assert(criticalInfo.curr_min_voltage > VOLT_MINIMUM); 
        assert(criticalInfo.curr_max_voltage < VOLT_MAXIMUM);
        // BMS has allowed for an unsafe voltage to be reached

        if (criticalInfo.curr_max_voltage 
                - criticalInfo.curr_min_voltage < BALANCED_VOLT_DIFF) {
            printf("Balanced successfully! in %f hours\n", 
                    ((float)i*(float)TIME_STEP_MS/(1000*60*60)));
            return; // Return, balanced successfully
        }
    }
}

void selfTest() {
    const double testVoltageF = 3.726;
    const uint16_t testVoltageUInt = (uint16_t)(testVoltageF*10000);
    double ampacitanceCalculated = voltToAmpacitance(testVoltageF);
    assert(eqWithTolF(ampacitanceCalculated, 7.27995, 0.3));
    double deltaVdeltaACalculated = deltaVDeltaAmpacity(ampacitanceCalculated);
    assert(eqWithTolF(deltaVdeltaACalculated, 0.33456, 0.002));
    uint16_t newVoltageCalculated = nextCellVoltage((uint16_t)testVoltageUInt,
            calculateDischargeCurrent(testVoltageF), TIME_STEP_MS);
    uint16_t newVoltageActual = 
        (uint16_t)((testVoltageF-0.00261556269333)*V_SCALE);
    assert(eqWithTolUI(newVoltageCalculated, newVoltageActual,
                newVoltageActual/10));
}
int main()
{
    selfTest();
    testCellBalance();
    return 0;
}
