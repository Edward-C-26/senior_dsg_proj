
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
 * - 1 x LTC6811
 * - 6 series cells used
 * - 2 thermistors total
 *   * thermistor 0 -> cells 1,2,3
 *   * thermistor 1 -> cells 4,5,6
 * - OV/UV support and passive balancing support
 */

// PEC = Packet Error Code, the CRC scheme used by the LTC6811 for data integrity verification
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
    // Put the high byte back in the upper half and merge in the low byte.
    return (uint16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

/*
 * Clear all discharge bits in the config structure after a write has gone out.
 * That way, the next balancing request starts from a clean slate instead of
 * accidentally reusing old cell selections.
 */
static void clear_discharge_bits(BMSConfigStructTypedef *cfg)
{
    for (uint8_t i = 0; i < 12U; i++) {
        // Clear every software discharge flag so old balancing requests do not linger.
        cfg->DischargeCell[i] = 0U;
    }
}

/*
 * Convert a raw auxiliary ADC reading into temperature using the project's lookup table.
 * If the reading falls outside the table's supported range, mark both fault
 * outputs so the caller knows the temperature result should not be trusted.
 */
static uint16_t convert_aux_code_to_temperature_mC(uint16_t aux_code, bool *dcFault, bool *tempFault)
{
    /*
     * lookupTableTemps[] is already defined in LTC6811.h in your current codebase.
     * We compute its size locally instead of relying on a separate macro.
     */
    // Figure out how many temperature entries exist in the lookup table.
    const int lookup_table_size = (int)(sizeof(lookupTableTemps) / sizeof(lookupTableTemps[0]));
    // Convert the raw ADC code into the scaled value used by the lookup table.
    double scaled = ((double)aux_code) / 100.0;
    // Shift the scaled value so it lines up with the first table entry.
    int index = (int)lround(scaled) - 21;

    if ((index < 0) || (index >= lookup_table_size)) {
        // Flag the reading as invalid when it falls outside the supported table range.
        *dcFault = true;
        *tempFault = true;
        return 0U;
    }

    // Clear the fault flags because this reading maps cleanly into the table.
    *dcFault = false;
    *tempFault = false;
    // Return the table temperature in milli-degrees Celsius to match the rest of the code.
    return (uint16_t)lookupTableTemps[index] * 1000U;
}

/* ---------- PEC ---------- */

//! @brief Initializes Packet Error Code LUT by generating PEC look up table -> call on startup
//! @returns none
void initPECTable(void)
{
    for (uint16_t i = 0; i < PEC_TABLE_SIZE; i++) {
        // Start each entry with the current byte shifted into the CRC working position.
        uint16_t remainder = (uint16_t)(i << 7);

        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((remainder & 0x4000U) != 0U) {
                // If the top CRC bit is set, shift and apply the LTC6811 polynomial.
                remainder = (uint16_t)((remainder << 1) ^ CRC15_POLY);
            } else {
                // Otherwise a plain left shift moves to the next CRC step.
                remainder = (uint16_t)(remainder << 1);
            }
        }

        // Save the finished CRC remainder so later PEC calculations are faster.
        pec15Table[i] = remainder;
    }
}

/*
 * Calculate the packet error code for a byte buffer using the LTC6811's CRC15
 * scheme.
 */
uint16_t calculatePEC(uint8_t len, uint8_t *data)
{
    // Seed the CRC with the LTC6811-defined starting value.
    uint16_t remainder = LTC6811_PEC_SEED;

    for (uint8_t i = 0; i < len; i++) {
        // Combine the next data byte with the current CRC state to index the lookup table.
        uint16_t address = (uint16_t)(((remainder >> 7) ^ data[i]) & 0x00FFU);
        // Advance the CRC using the lookup result instead of doing the bit math every time.
        remainder = (uint16_t)((remainder << 8) ^ pec15Table[address]);
    }

    // The LTC6811 expects the 15-bit PEC left-aligned in the returned 16-bit field.
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
    // Use a tiny software delay while toggling chip select to wake the LTC6811.
    volatile uint32_t delay = 1U;
    // Pull CS low to generate the wake pulse.
    HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
    while (delay--) {
        // Burn a couple of cycles so the pulse is not too short.
        __NOP();
    }
    // Release CS high again so normal SPI traffic can start.
    HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_SET);
}

/*
 * Send a command frame to every LTC device on the bus.
 */
void sendBroadcastCommand(CommandCodeTypedef command)
{
    // Build the 4-byte command frame: 2 command bytes followed by 2 PEC bytes.
    uint8_t cmd[4];
    uint16_t pec;

    // Split the command enum into the upper and lower command bytes.
    cmd[0] = (uint8_t)((command >> 8) & 0x0FU);
    cmd[1] = (uint8_t)(command & 0xFFU);

    // Compute the PEC over the command bytes before transmitting.
    pec = calculatePEC(2U, cmd);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    // Send the same command to every LTC6811 on the shared bus.
    SPIWrite(cmd, sizeof(cmd));
}

/*
 * Send a command frame to one addressed LTC device.
 *
 * The device address is folded into the command header before the PEC is generated.
 */
void sendAddressCommand(CommandCodeTypedef command, uint8_t address)
{
    // Build a command frame that targets one specific LTC6811 address.
    uint8_t cmd[4];
    uint8_t bytes[2];
    uint16_t pec;

    // Pack the address bits into the first command byte alongside the opcode bits.
    bytes[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((command >> 8) & 0x07U));
    bytes[1] = (uint8_t)(command & 0xFFU);

    // Copy the command header into the transmit buffer.
    cmd[0] = bytes[0];
    cmd[1] = bytes[1];

    // Generate the PEC from the addressed command header.
    pec = calculatePEC(2U, bytes);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    // Push the addressed command frame over SPI.
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
    // Prepare a full-duplex frame with room for the command, returned data, and returned PEC.
    uint8_t tx[LTC6811_READ_FRAME_BYTES] = {0};
    uint8_t rx[LTC6811_READ_FRAME_BYTES] = {0};
    uint8_t cmd_bytes[2];
    uint8_t pec_data[6];
    uint16_t expected_pec;
    uint16_t received_pec;

    // Pack the addressed read command exactly the way the LTC6811 expects it.
    cmd_bytes[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((command >> 8) & 0x07U));
    cmd_bytes[1] = (uint8_t)(command & 0xFFU);

    // Place the command header at the front of the transmit frame.
    tx[0] = cmd_bytes[0];
    tx[1] = cmd_bytes[1];

    // Append the PEC for the command so the LTC6811 will accept the request.
    expected_pec = calculatePEC(2U, cmd_bytes);
    tx[2] = (uint8_t)((expected_pec >> 8) & 0xFFU);
    tx[3] = (uint8_t)(expected_pec & 0xFFU);

    // Clock out the command while simultaneously capturing the device response.
    SPIWriteRead(tx, rx, sizeof(tx));

    for (uint8_t i = 0; i < 6U; i++) {
        // Copy just the returned register payload bytes for the PEC check.
        pec_data[i] = rx[4U + i];
    }

    // Recalculate the PEC over the returned data bytes.
    expected_pec = calculatePEC(6U, pec_data);
    // Rebuild the PEC that came back from the LTC6811.
    received_pec = (uint16_t)(((uint16_t)rx[10] << 8) | rx[11]);

    if (command == ReadCellVoltageRegisterGroup1to3) {
        // Group A holds cells 1 through 3.
        data[0] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[1] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[2] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadCellVoltageRegisterGroup4to6) {
        // Group B holds cells 4 through 6 for this 6-cell setup.
        data[3] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[4] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[5] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadAuxiliaryGroupA) {
        // Auxiliary group A carries the first three AUX readings.
        data[0] = ltc6811_bytes_to_u16(rx[4], rx[5]);
        data[1] = ltc6811_bytes_to_u16(rx[6], rx[7]);
        data[2] = ltc6811_bytes_to_u16(rx[8], rx[9]);
    } else if (command == ReadAuxiliaryGroupB) {
        // Auxiliary group B is only used here for the fourth AUX reading.
        data[3] = ltc6811_bytes_to_u16(rx[4], rx[5]);
    } else if (command == ReadConfigurationRegisterGroup) {
        // Return the raw configuration bytes so higher-level code can unpack them later.
        data[0] = (uint16_t)(((uint16_t)rx[4] << 8) | rx[5]);
        data[1] = (uint16_t)(((uint16_t)rx[6] << 8) | rx[7]);
        data[2] = (uint16_t)(((uint16_t)rx[8] << 8) | rx[9]);
        data[3] = (uint16_t)(((uint16_t)rx[10] << 8) | rx[11]);
    }

    // Only report success when the returned data PEC matches what we calculated locally.
    return (expected_pec == received_pec);
}

/* ---------- config ---------- */

/*
 * Pack the current software configuration into the exact byte layout expected
 * by the LTC6811 and write it to one addressed device.
 */
void writeConfigAddress(BMSConfigStructTypedef *cfg, uint8_t address)
{
    // Build the full addressed write frame: command, command PEC, config bytes, and data PEC.
    uint8_t cmd[12];
    uint16_t pec;
    // uint8_t dummy[8];

    // Put the addressed write-config opcode into the first two bytes.
    cmd[0] = (uint8_t)(0x80U | ((address << 3) & 0x78U) | ((WriteConfigurationRegisterGroup >> 8) & 0x07U));
    cmd[1] = (uint8_t)(WriteConfigurationRegisterGroup & 0xFFU);

    // Protect the command header with its own PEC.
    pec = calculatePEC(2U, cmd);
    cmd[2] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(pec & 0xFFU);

    // Pack the first config byte with GPIO pull-down settings and ADC/reference control bits.
    cmd[4] = (uint8_t)((cfg->GPIO5PulldownOff << 7) |
                       (cfg->GPIO4PulldownOff << 6) |
                       (cfg->GPIO3PulldownOff << 5) |
                       (cfg->GPIO2PulldownOff << 4) |
                       (cfg->GPIO1PulldownOff << 3) |
                       (cfg->ReferenceOn << 2) |
                       (cfg->ADCModeOption));

    // Store the lower 8 bits of the undervoltage threshold.
    cmd[5] = (uint8_t)(cfg->UndervoltageComparisonVoltage & 0xFFU);
    // Pack the upper UV bits together with the lower OV bits.
    cmd[6] = (uint8_t)(((cfg->OvervoltageComparisonVoltage << 4) & 0xF0U) |
                       ((cfg->UndervoltageComparisonVoltage >> 8) & 0x0FU));
    // Store the remaining OV threshold bits.
    cmd[7] = (uint8_t)((cfg->OvervoltageComparisonVoltage >> 4) & 0xFFU);

    // Pack discharge-enable bits for cells 1 through 8.
    cmd[8] = (uint8_t)((cfg->DischargeCell[7] << 7) |
                       (cfg->DischargeCell[6] << 6) |
                       (cfg->DischargeCell[5] << 5) |
                       (cfg->DischargeCell[4] << 4) |
                       (cfg->DischargeCell[3] << 3) |
                       (cfg->DischargeCell[2] << 2) |
                       (cfg->DischargeCell[1] << 1) |
                       (cfg->DischargeCell[0]));

    // Pack discharge-enable bits for cells 9 through 12 plus the discharge timeout.
    cmd[9] = (uint8_t)(((cfg->DischargeTimeoutValue << 4) & 0xF0U) |
                       (cfg->DischargeCell[11] << 3) |
                       (cfg->DischargeCell[10] << 2) |
                       (cfg->DischargeCell[9] << 1) |
                       (cfg->DischargeCell[8]));

    // Calculate the PEC for the six configuration bytes only.
    pec = calculatePEC(LTC6811_CFG_BYTES, &cmd[4]);
    cmd[10] = (uint8_t)((pec >> 8) & 0xFFU);
    cmd[11] = (uint8_t)(pec & 0xFFU);

    // Send the completed config frame to the selected LTC6811.
    SPIWrite(cmd, sizeof(cmd));
    //readConfig(address, dummy);
}

/*
 * Write the current config to every LTC device listed in the BMS config.
 */
void writeConfigAll(BMSConfigStructTypedef *cfg)
{
    // Wake the bus once before stepping through every LTC6811 board.
    wakeup_idle();

    for (uint8_t i = 0; i < cfg->numOfICs; i++) {
        // Reuse the same config struct while targeting each board address in turn.
        writeConfigAddress(cfg, cfg->address[i]);
    }
}

/*
 * Read back the configuration bytes from one LTC device.
 * This is useful as a sanity check after writing new settings.
 */
bool readConfig(uint8_t address, uint8_t cfg[6])
{
    uint16_t raw[3] = {0};
    bool ok;

    wakeup_idle();
    ok = readRegister(ReadConfigurationRegisterGroup, address, raw);

    cfg[0] = (uint8_t)((raw[0] >> 8) & 0xFFU);
    cfg[1] = (uint8_t)(raw[0] & 0xFFU);
    cfg[2] = (uint8_t)((raw[1] >> 8) & 0xFFU);
    cfg[3] = (uint8_t)(raw[1] & 0xFFU);
    cfg[4] = (uint8_t)((raw[2] >> 8) & 0xFFU);
    cfg[5] = (uint8_t)(raw[2] & 0xFFU);

    return ok;
}

/* ---------- cell voltage ---------- */

/*
 * Read the two voltage register groups that cover the six active cells in this
 * reduced pack layout.
 */
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[6])
{
    // Hold both voltage register groups before copying only the used cell entries out.
    uint16_t raw[12] = {0};
    bool ok_a;
    bool ok_b;

    // Read the first three cell voltages from register group A.
    ok_a = readRegister(ReadCellVoltageRegisterGroup1to3, address, raw);
    // Read the next three cell voltages from register group B.
    ok_b = readRegister(ReadCellVoltageRegisterGroup4to6, address, raw);

    for (uint8_t i = 0; i < LTC6811_USED_CELLS; i++) {
        // Copy the packed raw readings into the caller's 6-cell buffer.
        cellVoltage[i] = raw[i];
    }

    // Both register reads must pass PEC for the overall voltage read to count as valid.
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
    // Reuse one per-board buffer while walking through all secondary boards.
    uint16_t boardVoltage[6] = {0};
    bool dataValid = true;

    // Wake the chain before starting a new conversion cycle.
    wakeup_idle();
    HAL_Delay(2);

    // Clear old conversion results, then trigger a fresh cell-voltage conversion on every board.
    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellVoltageADCConversionAll);
    HAL_Delay(10);

    // Wake the parts again before starting the readback phase.
    wakeup_idle();
    HAL_Delay(2);

    for (uint8_t board = 0; board < NUM_BOARDS; board++) {
        // Read one board's six cell voltages and remember whether the PEC checked out.
        bool pec_ok = readCellVoltage(board, boardVoltage);
        dataValid &= pec_ok;

        for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
            // Convert the local cell index on this board into the flattened pack index.
            uint8_t globalCell = (uint8_t)(board * LTC6811_USED_CELLS + cell);

            // Store the newest measured voltage for this cell.
            bmsData[globalCell].voltage = boardVoltage[cell];

            if (pec_ok) {
                // Clear the PEC fault bit when this board's read was valid.
                bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            } else {
                // Set the PEC fault bit so higher-level logic knows this voltage read is suspect.
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
    // AUX group A is enough for this design because only two thermistors are used.
    uint16_t aux[4] = {0};
    bool ok;

    // Read the AUX channels tied to the thermistor divider circuits.
    ok = readRegister(ReadAuxiliaryGroupA, address, aux);

    /*
     * For your 6S design:
     * - AUX A channel 1 -> thermistor for cells 1-3
     * - AUX A channel 2 -> thermistor for cells 4-6
     */
    for (uint8_t i = 0; i < LTC6811_USED_THERMISTORS; i++) {
        // Convert each thermistor ADC code into temperature and its associated validity flags.
        cellTemp[i] = convert_aux_code_to_temperature_mC(aux[i], &dcFault[i], &tempFault[i]);
    }

    // Return whether the underlying AUX register read passed the PEC check.
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
    // Reuse one board-sized set of temperature and fault buffers across the loop.
    uint16_t boardTemp[2] = {0};
    bool boardDcFault[2] = {false, false};
    bool boardTempFault[2] = {false, false};
    bool dataValid = true;

    // Wake the LTC chain before starting a new AUX conversion.
    wakeup_idle();
    HAL_Delay(2);

    // Clear stale data, then start a new temperature conversion on every board.
    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellTempVoltageADCConversionAll);
    HAL_Delay(20);

    // Wake the parts again before reading the finished conversion results.
    wakeup_idle();
    HAL_Delay(2);

    for (uint8_t board = 0; board < NUM_BOARDS; board++) {
        // Read one board's thermistor values and remember whether the PEC was valid.
        bool pec_ok = readCellTemp(board, boardTemp, boardDcFault, boardTempFault);
        dataValid &= pec_ok;

        for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
            // Map the local board cell index into the flattened pack index.
            uint8_t globalCell = (uint8_t)(board * LTC6811_USED_CELLS + cell);
            // Cells 1-3 share thermistor 0 and cells 4-6 share thermistor 1.
            uint8_t therm_idx = (cell < 3U) ? 0U : 1U;

            // Copy the shared thermistor temperature onto each cell in that group.
            bmsData[globalCell].temperature = boardTemp[therm_idx];

            if (pec_ok) {
                // Clear the PEC fault when the AUX read came back clean.
                bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
            } else {
                // Mark the reading as suspect when the AUX PEC does not match.
                bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
            }

            if (boardTempFault[therm_idx]) {
                // Flag temperatures that fell outside the supported lookup range.
                bmsData[globalCell].fault |= CELL_TEMP_FAIL_MASK;
            } else {
                // Clear the temp fault once the mapped thermistor reading is valid again.
                bmsData[globalCell].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);
            }

            if (boardDcFault[therm_idx]) {
                // Carry forward the thermistor disconnect-style fault for all cells on that sensor.
                bmsData[globalCell].fault |= CELL_DCFAULT_MASK;
            } else {
                // Clear the disconnect-style fault when the thermistor reading looks healthy.
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
    // Wake the parts before pushing out any new balancing selections.
    wakeup_idle();

    for (uint8_t board = 0; board < cfg->numOfICs; board++) {
        for (uint8_t cell = 0; cell < 12U; cell++) {
            // Copy this board's requested discharge pattern into the config struct.
            cfg->DischargeCell[cell] = cellDischarge[board][cell];
        }

        // Write the updated discharge bits to the current board.
        writeConfigAddress(cfg, cfg->address[board]);
    }

    // Reset the software copy of the discharge bits so the next request starts clean.
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
    // Store the pulldown-test voltages for one board at a time.
    uint16_t adowVoltage[12] = {0};
    bool dataValid = true;

    // Wake the chain and clear any stale conversion state before the diagnostic.
    wakeup_idle();
    sendBroadcastCommand(ClearRegisters);

    /*
     * The repeated ADOW pulldown command pattern was retained from your original file.
     * This routine is still optional and mainly diagnostic.
     */
    for (uint8_t i = 0; i < 5U; i++) {
        // Repeat the pulldown conversion command to follow the existing open-wire routine.
        sendBroadcastCommand(StartOpenWireConversionPulldown);
    }

    // Give the open-wire conversion sequence time to finish.
    HAL_Delay(20);
    wakeup_idle();

    for (uint8_t board = 0; board < cfg.numOfICs; board++) {
        // Read back the pulldown-test voltages for this board.
        bool ok = readCellVoltage(cfg.address[board], adowVoltage);
        dataValid &= ok;

        for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
            // Convert the board-local index into the flattened pack index.
            uint8_t globalCell = (uint8_t)(board * cfg.numOfCellsPerIC + cell);
            // Compare the pulldown reading against the previously stored normal voltage.
            uint16_t storedVoltage = bmsData[globalCell].voltage;

            if ((storedVoltage > adowVoltage[cell]) &&
                ((storedVoltage - adowVoltage[cell]) >= 1000U)) {
                // A large drop during pulldown suggests an open connection on this cell input.
                bmsData[globalCell].fault |= CELL_DISCONNECT_MASK;
            } else {
                // Clear the disconnect flag when the pulldown result looks normal.
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
    // Hold just one board's voltage readings for this polling pass.
    uint16_t boardVoltage[6] = {0};
    bool dataValid;

    // Wake the addressed board and rewrite its config before starting the conversion.
    wakeup_idle();
    writeConfigAddress(cfg, cfg->address[board_num]);

    // Clear old data, then trigger a new voltage conversion across the chain.
    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellVoltageADCConversionAll);

    // Wait briefly for the conversion to complete, then wake the chain again for the read.
    HAL_Delay(3);
    wakeup_idle();
    HAL_Delay(1);

    // Read only the requested board's six cell voltages.
    dataValid = readCellVoltage(cfg->address[board_num], boardVoltage);

    for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
        // Convert the board-local index into the flattened pack index.
        uint8_t globalCell = (uint8_t)(board_num * LTC6811_USED_CELLS + cell);
        // Store the newest voltage for this cell.
        bmsData[globalCell].voltage = boardVoltage[cell];

        if (dataValid) {
            // Clear the PEC fault bit when this read succeeds.
            bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
        } else {
            // Set the PEC fault bit when the returned frame fails validation.
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
    // Hold one board's thermistor temperatures and fault flags for this poll.
    uint16_t boardTemp[2] = {0};
    bool boardDcFault[2] = {false, false};
    bool boardTempFault[2] = {false, false};
    bool dataValid;

    // Wake the chain and refresh this board's config before the AUX conversion.
    wakeup_idle();
    writeConfigAddress(cfg, cfg->address[board_num]);

    // Clear stale data, then start a fresh thermistor conversion.
    sendBroadcastCommand(ClearRegisters);
    sendBroadcastCommand(StartCellTempVoltageADCConversionAll);

    // Wait for the conversion to finish, then wake the chain for readback.
    HAL_Delay(3);
    wakeup_idle();
    HAL_Delay(1);

    // Read this board's thermistor channels and collect any associated fault flags.
    dataValid = readCellTemp(cfg->address[board_num], boardTemp, boardDcFault, boardTempFault);

    for (uint8_t cell = 0; cell < LTC6811_USED_CELLS; cell++) {
        // Convert the board-local cell index into the flattened pack index.
        uint8_t globalCell = (uint8_t)(board_num * LTC6811_USED_CELLS + cell);
        // Cells 1-3 use thermistor 0 and cells 4-6 use thermistor 1.
        uint8_t therm_idx = (cell < 3U) ? 0U : 1U;

        // Copy the shared thermistor temperature into this cell entry.
        bmsData[globalCell].temperature = boardTemp[therm_idx];

        if (dataValid) {
            // Clear the PEC fault bit when the AUX frame validates correctly.
            bmsData[globalCell].fault &= (uint8_t)(~CELL_PEC_FAIL_MASK);
        } else {
            // Set the PEC fault bit when the AUX frame fails its PEC check.
            bmsData[globalCell].fault |= CELL_PEC_FAIL_MASK;
        }

        if (boardTempFault[therm_idx]) {
            // Mark the cell when its shared thermistor temperature is out of valid range.
            bmsData[globalCell].fault |= CELL_TEMP_FAIL_MASK;
        } else {
            // Clear the temperature fault once the shared sensor returns to normal.
            bmsData[globalCell].fault &= (uint8_t)(~CELL_TEMP_FAIL_MASK);
        }

        if (boardDcFault[therm_idx]) {
            // Mark the cell when its shared thermistor reading indicates a disconnect-style issue.
            bmsData[globalCell].fault |= CELL_DCFAULT_MASK;
        } else {
            // Clear that fault once the thermistor input looks valid again.
            bmsData[globalCell].fault &= (uint8_t)(~CELL_DCFAULT_MASK);
        }
    }

    return dataValid;
}
