# README

This folder contains all development firmwares.

For the **most up to date** firmware. Use firmware starting with `LORA_FLIGHT_FIRMWARE_V...`

Any project not named below is unknown or was simply a quick test.

### LORATESTING
SPI functions working at a basic level for LoRa on SPI

### LoRa_Board_I2C_Test
This firmware was a failed attempt at using the MSP430's included I2C module. This was not working as desired so we moved
to a bitbanged version instead.

### LoRa_Board_Serial_Print
This was a test of general MSP430 UART in our context

### LoRa_Board_UART_Test
This is the **full working GPS via UART code**. It **sets the gps settings** via UART then **sets to our struct for sending**.

### LoRa_Flight_Firmware_V1
**THIS IS THE MAIN FIRMWARE TO USE** It
1. Sets up all of the modules/ports/configures sensors/clocks, etc.
2. Gathers all the sensor data and sets to struct
3. Gathers all the GPS data and sets to struct
4. Sends all the data via LoRa and handles LoRa-isms

### LoRa_GPS_Only_Test
This is an **isolated test of the GPS sending via LoRa** for debugging/developing GPS specific things.

### LoRa_I2C_BitBang
This is the **working I2C code for the project**. It **gathers all the I2C sensor data** (currently all sensors minus the GPS) and **sets these to the struct for sending**.

### LoRa_Radio_TX_Test
This is a working send of sample data on the LoRa Radio that can be simply received by any groundstation that is properly configured, in our case we used the featherboard m0.

### LoRa_Sample_Struct_Payload
This is an instance of the struct for reference to the LoRa payload *note:* **TODO** we didn't get the GPS data working in the struct, we had to use something seprate. This could be worked on in the future.

### goats
Used for playing around with figuring out LoRa settings
