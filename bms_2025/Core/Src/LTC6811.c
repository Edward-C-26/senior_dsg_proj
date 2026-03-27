
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "LTC6811.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * Rewritten for Project 35 scaled configuration:
 * - 1 x LTC6811
 * - 6 series cells used
 * - 2 thermistors total
 *   * thermistor 0 -> cells 1,2,3
 *   * thermistor 1 -> cells 4,5,6
 * - discharge monitoring only (no charger/CAN assumptions here)
 * - retains OV/UV support and passive balancing support
 */

#define PEC_TABLE_SIZE              256U
#define LTC6811_USED_CELLS          6U
#define LTC6811_USED_THERMISTORS    2U
#define LTC6811_CFG_BYTES           6U
#define LTC6811_READ_FRAME_BYTES    12U
#define LTC6811_PEC_SEED            16U
#define CRC15_POLY                  0x4599U

static uint16_t pec15Table[PEC_TABLE_SIZE];

/* ---------- local helpers ---------- */

/*
 * Combine the low and high bytes returned by the LTC6811 into one 16-bit
 * register value.
 */
static uint16_t ltc6811_bytes_to_u16(uint8_t low, uint8_t high)
{
    return (uint16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

/*
 * Clear all discharge bits in the config structure after a write has gone out.
 *
 * That way, the next balancing request starts from a clean slate instead of
 * accidentally reusing old cell selections.
 */
static void clear_discharge_bits(BMSConfigStructTypedef *cfg)
{
    for (uint8_t i = 0; i < 12U; i++) {
        cfg->DischargeCell[i] = 0U;
    }
}

/*
 * Convert a raw auxiliary ADC reading into temperature using the project's
 * lookup table.
 *
 * If the reading falls outside the table's supported range, mark both fault
 * outputs so the caller knows the temperature result should not be trusted.
 */
static uint16_t convert_aux_code_to_temperature_mC(uint16_t aux_code, bool *dcFault, bool *tempFault)
{
    /*
     * lookupTableTemps[] is already defined in LTC6811.h in your current codebase.
     * We compute its size locally instead of relying on a separate macro.
     */
    const int lookup_table_size = (int)(sizeof(lookupTableTemps) / sizeof(lookupTableTemps[0]));
    double scaled = ((double)aux_code) / 100.0;
    int index = (int)lround(scaled) - 21;

    if ((index < 0) || (index >= lookup_table_size)) {
        *dcFault = true;
        *tempFault = true;
        return 0U;
    }

    *dcFault = false;
    *tempFault = false;
    return (uint16_t)lookupTableTemps[index] * 1000U;
}

/* ---------- PEC ---------- */

/*
 * Precompute the PEC lookup table used by the LTC6811 packet format.
 *
 * Doing this once at startup is much cheaper than recomputing the polynomial
 * bit-by-bit for every command and response.
 */
void initPECTable(void)
{
    for (uint16_t i = 0; i < PEC_TABLE_SIZE; i++) {
        uint16_t remainder = (uint16_t)(i << 7);

        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((remainder & 0x4000U) != 0U) {
                remainder = (uint16_t)((remainder << 1) ^ CRC15_POLY);
            } else {
                remainder = (uint16_t)(remainder << 1);
            }
        }

        pec15Table[i] = remainder;
    }
}

/*
 * Calculate the packet error code for a byte buffer using the LTC6811's CRC15
 * scheme.
 */
uint16_t calculatePEC(uint8_t len, uint8_t *data)
{
    uint16_t remainder = LTC6811_PEC_SEED;

    for (uint8_t i = 0; i < len; i++) {
        uint16_t address = (uint16_t)(((remainder >> 7) ^ data[i]) & 0x00FFU);
        remainder = (uint16_t)((remainder << 8) ^ pec15Table[address]);
    }

    return (uint16_t)(remainder << 1);
}

/* ---------- low-level command helpers ---------- */

/*
 * Wake the LTC6811 by pulsing chip select.
 *
 * The part can sit idle between transactions, so many call sites use this as
 * the first step before issuing a command.
 */
void wakeup_idle(void)
{
    volatile uint32_t delay = 1U;
    HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
    while (delay--) {
        __NOP();
    }
    HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);
}

/*
 * Send a command frame to every LTC device on the bus.
 */
void sendBroadcastCommand(CommandCodeTypedef command)
{
    uint8_t cmd[4];
    uint16_t pec;

    cmd[0] = (uint8_t)((command >> 8) & 0x0FU);
    cmd[1] = (uint8_t)(command & 0xFFU);

    pec = calculatePEC(2U, cmd);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    SPIWrite(cmd, sizeof(cmd));
}

/*
 * Send a command frame to one addressed LTC device.
 *
 * The device address is folded into the command header before the PEC is
 * generated.
 */
void sendAddressCommand(CommandCodeTypedef command, uint8_t address)
{
    uint8_t cmd[4];
    uint8_t bytes[2];
    uint16_t pec;

    bytes[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((command >> 8) & 0x07U));
    bytes[1] = (uint8_t)(command & 0xFFU);

    cmd[0] = bytes[0];
    cmd[1] = bytes[1];

    pec = calculatePEC(2U, bytes);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    SPIWrite(cmd, sizeof(cmd));
}

/*
 * Read one register group from the addressed LTC device and unpack the
 * returned bytes into the caller's data buffer.
 *
 * For normal data reads, the function verifies the returned PEC so the caller
 * can tell whether the SPI transaction was trustworthy.
 */
bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data)
{
    uint8_t tx[LTC6811_READ_FRAME_BYTES] = {0};
    uint8_t rx[LTC6811_READ_FRAME_BYTES] = {0};
    uint8_t cmd_bytes[2];
    uint8_t pec_data[6];
    uint16_t expected_pec;
    uint16_t received_pec;

    cmd_bytes[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((command >> 8) & 0x07U));
    cmd_bytes[1] = (uint8_t)(command & 0xFFU);

    tx[0] = cmd_bytes[0];
    tx[1] = cmd_bytes[1];

    expected_pec = calculatePEC(2U, cmd_bytes);
    tx[2] = (uint8_t)((expected_pec >> 8) & 0xFFU);
    tx[3] = (uint8_t)(expected_pec & 0xFFU);

    SPIWriteRead(tx, rx, sizeof(tx));

    for (uint8_t i = 0; i < 6U; i++) {
        pec_data[i] = rx[4U + i];
    }

    expected_pec = calculatePEC(6U, pec_data);
    received_pec = (uint16_t)(((uint16_t)rx[10] << 8) | rx[11]);

    if (command == ReadCellVoltageRegisterGroup1to3) {
        data[0] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[1] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[2] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadCellVoltageRegisterGroup4to6) {
        data[3] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[4] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[5] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadCellVoltageRegisterGroup7to9) {
        data[6] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[7] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[8] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadCellVoltageRegisterGroup10to12) {
        data[9]  = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[10] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[11] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadAuxiliaryGroupA) {
        data[0] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[1] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[2] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadAuxiliaryGroupB) {
        data[3] = ltc6811_bytes_to_u16(rx[4], rx[5]);
    } else if (command == ReadConfigurationRegisterGroup) {
        data[0] = (uint16_t)(((uint16_t)rx[4] << 8) | rx[5]);
        data[1] = (uint16_t)(((uint16_t)rx[6] << 8) | rx[7]);
        data[2] = (uint16_t)(((uint16_t)rx[8] << 8) | rx[9]);
        data[3] = (uint16_t)(((uint16_t)rx[10] << 8) | rx[11]);
        return true; /* config path is kept as-is */
    }

    return (expected_pec == received_pec);
}

/* ---------- config ---------- */

/*
 * Pack the current software configuration into the exact byte layout expected
 * by the LTC6811 and write it to one addressed device.
 */
void writeConfigAddress(BMSConfigStructTypedef *cfg, uint8_t address)
{
    uint8_t cmd[12];
    uint16_t pec;
    uint8_t dummy[8];

    cmd[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((WriteConfigurationRegisterGroup >> 8) & 0x07U));
    cmd[1] = (uint8_t)(WriteConfigurationRegisterGroup & 0xFFU);

    pec = calculatePEC(2U, cmd);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    cmd[4] = (uint8_t)((cfg->GPIO5PulldownOff << 7) |
                       (cfg->GPIO4PulldownOff << 6) |
                       (cfg->GPIO3PulldownOff << 5) |
                       (cfg->GPIO2PulldownOff << 4) |
                       (cfg->GPIO1PulldownOff << 3) |
                       (cfg->ReferenceOn << 2) |
                       (cfg->ADCModeOption));

    cmd[5] = (uint8_t)(cfg->UndervoltageComparisonVoltage & 0xFFU);
    cmd[6] = (uint8_t)(((cfg->OvervoltageComparisonVoltage << 4) & 0xF0U) |
                       ((cfg->UndervoltageComparisonVoltage >> 8) & 0x0FU));
    cmd[7] = (uint8_t)((cfg->OvervoltageComparisonVoltage >> 4) & 0xFFU);

    cmd[8] = (uint8_t)((cfg->DischargeCell[7] << 7) |
                       (cfg->DischargeCell[6] << 6) |
                       (cfg->DischargeCell[5] << 5) |
                       (cfg->DischargeCell[4] << 4) |
                       (cfg->DischargeCell[3] << 3) |
                       (cfg->DischargeCell[2] << 2) |
                       (cfg->DischargeCell[1] << 1) |
                       (cfg->DischargeCell[0]));

    cmd[9] = (uint8_t)(((cfg->DischargeTimeoutValue << 4) & 0xF0U) |
                       (cfg->DischargeCell[11] << 3) |
                       (cfg->DischargeCell[10] << 2) |
                       (cfg->DischargeCell[9] << 1) |
                       (cfg->DischargeCell[8]));

    pec = calculatePEC(LTC6811_CFG_BYTES, &cmd[4]);
    cmd[10] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[11] = (uint8_t)(pec & 0xFFU);

    SPIWrite(cmd, sizeof(cmd));
    readConfig(address, dummy);
}

/*
 * Write the current config to every LTC device listed in the BMS config.
 */
void writeConfigAll(BMSConfigStructTypedef *cfg)
{
    wakeup_idle();

    for (uint8_t i = 0; i < cfg->numOfICs; i++) {
        writeConfigAddress(cfg, cfg->address[i]);
    }
}

/*
 * Read back the configuration bytes from one LTC device.
 *
 * This is useful as a sanity check after writing new settings.
 */
bool readConfig(uint8_t address, uint8_t cfg[8])
{
    uint16_t raw[4] = {0};
    bool ok;

    wakeup_idle();
    ok = readRegister(ReadConfigurationRegisterGroup, address, raw);

    cfg[0] = (uint8_t)((raw[0] >> 8) & 0xFFU);
    cfg[1] = (uint8_t)(raw[0] & 0xFFU);
    cfg[2] = (uint8_t)((raw[1] >> 8) & 0xFFU);
    cfg[3] = (uint8_t)(raw[1] & 0xFFU);
    cfg[4] = (uint8_t)((raw[2] >> 8) & 0xFFU);
    cfg[5] = (uint8_t)(raw[2] & 0xFFU);
    cfg[6] = (uint8_t)((raw[3] >> 8) & 0xFFU);
    cfg[7] = (uint8_t)(raw[3] & 0xFFU);

    return ok;
}

/* ---------- cell voltage ---------- */

/*
 * Read the two voltage register groups that cover the six active cells in this
 * reduced pack layout.
 */
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[6])
{
    uint16_t raw[12] = {0};
    bool ok_a;
    bool ok_b;

    ok_a = readRegister(ReadCellVoltageRegisterGroup1to3, address, raw);
    ok_b = readRegister(ReadCellVoltageRegisterGroup4to6, address, raw);

    for (uint8_t i = 0; i < LTC6811_USED_CELLS; i++) {
        cellVoltage[i] = raw[i];
    }

    return (ok_a && ok_b);
}

/*
 * Start a fresh voltage conversion on every board and copy the returned values
 * into the shared CellData array used by the rest of the application.
 *
 * The per-cell PEC fault bit is refreshed at the same time so later logic can
 * tell whether the most recent read was valid.
 */
bool readAllCellVoltages(CellData bmsData[])
{
    uint16_t boardVoltage[6] = {0};
    bool dataValid = true;

    wakeup_idle();
    HAL_Delay(2);

    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellVoltageADCConversionAll);
    HAL_Delay(10);

    wakeup_idle();
    HAL_Delay(2);

    for (uint8_t board = 0; board < NUM_BOARDS; board++) {
        bool pec_ok = readCellVoltage(board, boardVoltage);
        dataValid &= pec_ok;

        for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
            uint8_t globalCell = (uint8_t)(board * LTC6811_USED_CELLS + cell);

            bmsData[globalCell].voltage = boardVoltage[cell];

            if (pec_ok) {
                bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            } else {
                bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
            }
        }
    }

    return dataValid;
}

/* ---------- temperature ---------- */

/*
 * Read the auxiliary channels used as thermistor inputs and convert them into
 * milli-degree Celsius values.
 *
 * This project maps one thermistor to cells 1-3 and the other to cells 4-6.
 */
bool readCellTemp(uint8_t address, uint16_t cellTemp[2], bool dcFault[2], bool tempFault[2])
{
    uint16_t aux[4] = {0};
    bool ok;

    ok = readRegister(ReadAuxiliaryGroupA, address, aux);

    /*
     * For your 6S design:
     * - AUX A channel 1 -> thermistor for cells 1-3
     * - AUX A channel 2 -> thermistor for cells 4-6
     */
    for (uint8_t i = 0; i < LTC6811_USED_THERMISTORS; i++) {
        cellTemp[i] = convert_aux_code_to_temperature_mC(aux[i], &dcFault[i], &tempFault[i]);
    }

    return ok;
}

/*
 * Start a fresh temperature conversion on every board and map the resulting
 * thermistor values onto each cell entry in the shared CellData array.
 *
 * Because multiple cells share each thermistor, the same temperature is copied
 * into each cell in that group, along with any corresponding fault flags.
 */
bool readAllCellTemps(CellData bmsData[])
{
    uint16_t boardTemp[2] = {0};
    bool boardDcFault[2] = {false, false};
    bool boardTempFault[2] = {false, false};
    bool dataValid = true;

    wakeup_idle();
    HAL_Delay(2);

    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellTempVoltageADCConversionAll);
    HAL_Delay(20);

    wakeup_idle();
    HAL_Delay(2);

    for (uint8_t board = 0; board < NUM_BOARDS; board++) {
        bool pec_ok = readCellTemp(board, boardTemp, boardDcFault, boardTempFault);
        dataValid &= pec_ok;

        for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
            uint8_t globalCell = (uint8_t)(board * LTC6811_USED_CELLS + cell);
            uint8_t therm_idx = (cell < 3U) ? 0U : 1U;

            bmsData[globalCell].temperature = boardTemp[therm_idx];

            if (pec_ok) {
                bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            } else {
                bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
            }

            if (boardTempFault[therm_idx]) {
                bmsData[globalCell].fault |= CELL_TEMP_FAIL_MASK;
            } else {
                bmsData[globalCell].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);
            }

            if (boardDcFault[therm_idx]) {
                bmsData[globalCell].fault |= CELL_DCFAULT_MASK;
            } else {
                bmsData[globalCell].fault &= (uint8_t)(~CELL_DCFAULT_MASK);
            }
        }
    }

    return dataValid;
}

/* ---------- balancing ---------- */

/*
 * Push the caller's requested discharge pattern out to the LTC6811 config
 * registers for each board.
 *
 * After the write completes, the temporary discharge bits inside the config
 * struct are cleared so the next call starts fresh.
 */
bool dischargeCellGroups(BMSConfigStructTypedef *cfg, bool cellDischarge[12][12])
{
    wakeup_idle();

    for (uint8_t board = 0; board < cfg->numOfICs; board++) {
        for (uint8_t cell = 0; cell < 12U; cell++) {
            cfg->DischargeCell[cell] = cellDischarge[board][cell];
        }

        writeConfigAddress(cfg, cfg->address[board]);
    }

    clear_discharge_bits(cfg);
    return true;
}

/* ---------- optional/open-wire ---------- */

/*
 * Run the open-wire diagnostic sequence and compare the result against the
 * stored normal-voltage readings.
 *
 * A significant drop during the pulldown test is treated as a likely open cell
 * connection and sets the disconnect fault bit for that cell.
 */
bool checkAllCellConnections(BMSConfigStructTypedef cfg, CellData bmsData[])
{
    uint16_t adowVoltage[12] = {0};
    bool dataValid = true;

    wakeup_idle();
    sendBroadcastCommand(ClearRegisters);

    /*
     * The repeated ADOW pulldown command pattern was retained from your original file.
     * This routine is still optional and mainly diagnostic.
     */
    for (uint8_t i = 0; i < 5U; i++) {
        sendBroadcastCommand(StartOpenWireConversionPulldown);
    }

    HAL_Delay(20);
    wakeup_idle();

    for (uint8_t board = 0; board < cfg.numOfICs; board++) {
        bool ok = readCellVoltage(cfg.address[board], adowVoltage);
        dataValid &= ok;

        for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
            uint8_t globalCell = (uint8_t)(board * cfg.numOfCellsPerIC + cell);
            uint16_t storedVoltage = bmsData[globalCell].voltage;

            if ((storedVoltage > adowVoltage[cell]) &&
                ((storedVoltage - adowVoltage[cell]) >= 1000U)) {
                bmsData[globalCell].fault |= CELL_DISCONNECT_MASK;
            } else {
                bmsData[globalCell].fault &= (uint8_t)(~CELL_DISCONNECT_MASK);
            }
        }
    }

    return dataValid;
}

/* ---------- single-board poll helpers used by main ---------- */

/*
 * Poll one board's cell voltages and copy them into the global CellData array
 * used by main.c.
 */
bool poll_single_secondary_voltage_reading(uint8_t board_num,
                                          BMSConfigStructTypedef *cfg,
                                          CellData bmsData[])
{
    uint16_t boardVoltage[6] = {0};
    bool dataValid;

    wakeup_idle();
    writeConfigAddress(cfg, cfg->address[board_num]);

    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellVoltageADCConversionAll);

    HAL_Delay(3);
    wakeup_idle();
    HAL_Delay(1);

    dataValid = readCellVoltage(cfg->address[board_num], boardVoltage);

    for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
        uint8_t globalCell = (uint8_t)(board_num * LTC6811_USED_CELLS + cell);
        bmsData[globalCell].voltage = boardVoltage[cell];

        if (dataValid) {
            bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
        } else {
            bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
        }
    }

    return dataValid;
}

/*
 * Poll one board's thermistor channels and update the corresponding cell
 * entries in the shared CellData array.
 *
 * This helper also refreshes the PEC, dc-fault, and temp-fault bits so the
 * fault logic can work from a single source of truth.
 */
bool poll_single_secondary_temp_reading(uint8_t board_num,
                                       BMSConfigStructTypedef *cfg,
                                       CellData bmsData[])
{
    uint16_t boardTemp[2] = {0};
    bool boardDcFault[2] = {false, false};
    bool boardTempFault[2] = {false, false};
    bool dataValid;

    wakeup_idle();
    writeConfigAddress(cfg, cfg->address[board_num]);

    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellTempVoltageADCConversionAll);

    HAL_Delay(3);
    wakeup_idle();
    HAL_Delay(1);

    dataValid = readCellTemp(cfg->address[board_num], boardTemp, boardDcFault, boardTempFault);

    for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
        uint8_t globalCell = (uint8_t)(board_num * LTC6811_USED_CELLS + cell);
        uint8_t therm_idx = (cell < 3U) ? 0U : 1U;

        bmsData[globalCell].temperature = boardTemp[therm_idx];

        if (dataValid) {
            bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
        } else {
            bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
        }

        if (boardTempFault[therm_idx]) {
            bmsData[globalCell].fault |= CELL_TEMP_FAIL_MASK;
        } else {
            bmsData[globalCell].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);
        }

        if (boardDcFault[therm_idx]) {
            bmsData[globalCell].fault |= CELL_DCFAULT_MASK;
        } else {
            bmsData[globalCell].fault &= (uint8_t)(~CELL_DCFAULT_MASK);
        }
    }

    return dataValid;
}
