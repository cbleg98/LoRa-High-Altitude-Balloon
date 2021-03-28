/** @file I2C_defs.h
 * 
 * @brief I2C headerfile for readability
 *
 */ 

#ifndef I2C_DEFS_H
#define I2C_DEFS_H

//I2C definitions for readability
#define SCL BIT7
#define SDA BIT6
#define WRITE_MODE 0
#define READ_MODE 1
#define EXT_TEMP_ADDR 0x4A
#define INT_TEMP_ADDR 0x4B
#define ACCEL_ADDR 0x1D
#define PRESSURE_ADDR 0x77
#define ACCEL_X_LSB 0x28
#define ACCEL_X_MSB 0x29
#define ACCEL_Y_LSB 0x2A
#define ACCEL_Y_MSB 0x2B
#define ACCEL_Z_LSB 0x2C
#define ACCEL_Z_MSB 0x2D
#define NAK_FLAG 0x01
#define I2C_DELAY 5
#define TEMP_MSB 0x00
#define TEMP_LSB 0x01
#define TEMP_CONFIG 0x03
#define GET_PRESSURE 0x48
#define GET_PRESS_TEMP 0x58
#define READ_PRESS 0x00

#endif /* I2C_DEFS_H */

/*** end of file ***/
