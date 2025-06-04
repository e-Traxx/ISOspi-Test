#ifndef ADBMS6830_H
#define ADBMS6830_H

#include <stdint.h>

// ADBMS6830 Command Definitions
#define WRCFGA   0x0001  // Write Configuration Group A
#define WRCFGB   0x0002  // Write Configuration Group B
#define RDCVA    0x0004  // Read Spare Voltage Register A
#define RDSVA    RDCVA   // alias
#define RDCVB    0x0006  // Read Spare Voltage Register B
#define RDSVB    RDCVB
#define RDCVC    0x0008  // Read Spare Voltage Register C
#define RDSVC    RDCVC
#define RDCVD    0x000A  // Read Spare Voltage Register D
#define RDSVD    RDCVD
#define RDCVE    0x000C  // Read Spare Voltage Register E
#define RDSVE    RDCVE
#define RDCVF    0x000E  // Read Spare Voltage Register F
#define RDSVF    RDCVF

#define RDCVA   0x0004
#define RDCVB   0x0006
#define RDCVC   0x0008
#define RDCVD   0x000A
#define RDCVE   0x000C
#define RDCVF   0x000E

#define RDAUXA  0x0010  // Read AUX Register A
#define RDAUXB  0x0012  // Read AUX Register B
#define RDAUXC  0x0014  // Read AUX Register C
#define RDAUXD  0x0016  // Read AUX Register D

#define ADCV    0x0020  // Start Cell ADC Conversion
#define CLRCMFLAG 0x0030 // Clear Sticky Flags
#define CLOVUV    0x0032 // Clear OV/UV Flags

#define STCOMM  0x0040  // Start/Stop Communication Command

// Macro to load a 16-bit command into a 2-byte array
#define load_array(array, value) \
    do { \
        (array)[0] = (uint8_t)(((value) >> 8) & 0xFF); \
        (array)[1] = (uint8_t)((value) & 0xFF); \
    } while (0)

// Function prototypes
void gen_crc_15(void);
uint16_t get_cmd_pec(const uint8_t *data, uint8_t len);
void gen_crc_10(void);
uint16_t get_data_pec(const uint8_t *data, uint8_t rx, uint8_t len);

void isospi_wake(void);
void Poll_ADBMS6830(const uint8_t *tx_cmd);
void write_ADBMS8630(uint8_t total_ic, const uint8_t tx_cmd[2], const uint8_t *data);
int8_t Read_ADBMS8630(uint8_t total_ic, const uint8_t tx_cmd[2], uint8_t *rx_data);
int8_t Read_CellVoltage_ADBMS8630(uint8_t reg, uint8_t total_ic, uint16_t cell_codes[][16]);
void Read_CellVoltage_Reg_ADBMS8630(uint8_t reg, uint8_t total_ic, uint8_t *data);
void get_spare_voltages(uint32_t *spare_volts);

#endif // ADBMS6830_H
