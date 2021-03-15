/** @file reg_map.h
 * 
 * @brief the register mapping for the RF95 LoRa Radio. 
 *
 */ 

#ifndef REG_MAP_H
#define REG_MAP_H

// Bits
#define SCLK    BIT1
#define MOSI    BIT3
#define MISO    BIT2
#define NSS     BIT0

// Registers
#define FIFO_00              0x00
#define OPMODE_01            0x01
#define FR_MSB_06            0x06
#define FR_MID_07            0x07
#define FR_LSB_08            0x08
#define POW_CONFIG_09        0x09
#define FIFO_ADDR_PTR_0D     0x0D
#define FIFO_TX_BASE_ADDR_0E 0x0E
#define FIFO_RX_BASE_ADDR_0F 0x0F
#define MODEM_CONFIG_1_1D    0x1D
#define MODEM_CONFIG_2_1E    0x1E
#define MODEM_CONFIG_3_26    0x26
#define PREAMBLE_LEN_MSB_20  0x20
#define PREAMBLE_LEN_LSB_21  0x21
#define PAYLOAD_LEN_22       0x22
#define MAX_PAYLOAD_LEN_23   0x23

//OPMODES
#define MODE_LORA_SLEEP 0x80
#define MODE_LORA_STBY  0x81
#define MODE_LORA_TX    0x83

#endif /* REG_MAP_H */

/*** end of file ***/
