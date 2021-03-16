/* LoRa_Flight_Firmware_V1/main.c
 * This is the MAIN firmware (version 1) for running this firmware on the MSP430FR2355 based board
 * 
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 * @author Larson Brandstetter
 */

#include <msp430.h> 
#include "reg_map.h"
#include "I2C_defs.h"

//------Configurable Parameters------
#define PAYLOAD_LEN 0x79
#define I2C_DELAY 5
#define SPI_DELAY 20

//----PAYLOAD STRUCT TO SEND DATA TO LORA-----
struct payload{
    long LORA_pressure;
    int LORA_int_temp;
    int LORA_ext_temp;
    int LORA_accel_x;
    int LORA_accel_y;
    int LORA_accel_z;
    //char LORA_GPS[100]; //uncomment if using this in the debounce loop
};

//--------------Globals--------------------
char j, k, h;
char i2c_status = 0x00; // 0b00000000 -- | 0 | 0 | 0 | 0 || 0 | 0 | 0 | 1-NAK | (currently unused)
// char temp_config_int, temp_config_ext; //uncomment this line if uncommenting the lines that get the config of the temp sensors
int i;
int int_temp, ext_temp, accel_x, accel_y, accel_z = 0; //actual temperature (internal and external) and acceleration values on each axis
unsigned int c[8]; //coefficients for the pressure sensor
unsigned long digital_press, digital_press_temp = 0; //these are the variables that are used in calculating the pressure, not the actual pressure values
long dT, press_temp, pressure = 0; //actual temperature from pressure sensor and pressure
long long OFF, SENS = 0; //used for pressure calculation

//-----GPS Globals-----
// The below character arrays are the strings needing to be sent to the GPS to properly configure the messages
char diable_GLL[] = {'$','P','U','B','X',',','4','0',',','G','L','L',',','0',',','0',',','0',',','0',',','0',',','0','*','5','C','\r','\n'};
char diable_GSV[] = {'$','P','U','B','X',',','4','0',',','G','S','V',',','0',',','0',',','0',',','0',',','0',',','0','*','5','9','\r','\n'};
char diable_GSA[] = {'$','P','U','B','X',',','4','0',',','G','S','A',',','0',',','0',',','0',',','0',',','0',',','0','*','4','E','\r','\n'};
char diable_VTG[] = {'$','P','U','B','X',',','4','0',',','V','T','G',',','0',',','0',',','0',',','0',',','0',',','0','*','5','E','\r','\n'};
char diable_RMC[] = {'$','P','U','B','X',',','4','0',',','R','M','C',',','0',',','0',',','0',',','0',',','0',',','0','*','4','7','\r','\n'};
char GPS_GNGGA[100]; //this holds the char array we want

//----LORA STRUCT----
struct payload LORA_PAYLOAD; //THIS IS THE LORA PAYLOAD ALL MEMBERS ARE DAYA TO BE SENT TO THE LORA, GLOBAL FOR INTERRUPT ACCESS

//--------------Functions--------------------

//-----SPI-----

//Delay for SPI
void delay(unsigned int j){
    unsigned int l;
    for(l = 0; l < j; l++){
        P6OUT ^= BIT2; //Toggle LED3
    }
}

void SPI_tx(char addr, char data){
    char temp;
    temp = 0x80 + addr; //0x80 is the send bit for SPI
    P4OUT &= ~NSS;
    UCA1TXBUF = temp;
    UCA1TXBUF = data;
    delay(SPI_DELAY);
    P4OUT |= NSS;
}

//currently unused
/*void SPI_burst_tx(char data[20]){
    for(i=0; i < PAYLOAD_LEN; i=i+2){
        UCA1TXBUF = data[i];
        delay(SPI_DELAY);
        UCA1TXBUF = data[i+1];
        delay(SPI_DELAY);
    }
    delay(PAYLOAD_LEN*SPI_DELAY);
}*/

char SPI_read_reg(char addr){
    char data;
    P4OUT &= ~NSS;
    UCA1TXBUF = addr;
    UCA1TXBUF = addr;
    data = UCA1RXBUF;
    delay(SPI_DELAY);
    P4OUT |= NSS;
    return data;
}

void SPI_tx_two_bytes(int val){
    UCA1TXBUF = (val >> 8);
    delay(SPI_DELAY);
    UCA1TXBUF = (val & 0x00FF);
    delay(SPI_DELAY);
}

//-----I2C-----

//cycle the i2c clock line high then low
void i2c_clock_cycle(void){
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SCL; //SCL LOW
    P6OUT ^= BIT0; //Toggle LED3
}

//transmit a high bit on the i2c line
void i2c_tx_high_bit(void){
    P4OUT |= SDA; //SDA HIGH
    //toggle clock
    i2c_clock_cycle();
}

//transit a low bit on the i2c line
void i2c_tx_low_bit(void){
    P4OUT &= ~SDA; //SDA LOW
    //toggle clock
    i2c_clock_cycle();
}

//transmit the start condition on the i2c line, handling any starting position of the lines and setting the lines ready for the first bit
void i2c_start(void){
    P4OUT |= SDA; //SDA HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SCL; //SCL LOW
}

//transmit the stop condition on the i2c line, handling the starting position of the lines and setting the lines to "idle"
void i2c_stop(void){
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SDA; //SDA HIGH
}

//transmit a nack condition on the i2c line
void i2c_nack(void){
    P4OUT |= SDA; //SDA HIGH
    for(j=0; j<I2C_DELAY; j++){}
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SDA; //SDA LOW
}

//transmit an ack on the i2c line
void i2c_ack(void){
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
}

//read an ack from the slave and do something. Right now all we do is set a flag which is never cleared
void i2c_read_ack(void){
    P4DIR &= ~SDA; // SDA input port
    for(j=0; j<I2C_DELAY*2; j++){}
    if(P4IN & SDA){
        i2c_status |= NAK_FLAG;
    }
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
    P4DIR |= SDA; // SDA output port
    for(j=0; j<I2C_DELAY; j++){}
}

//given any byte, transmit it on the i2c line
void i2c_tx(char byte){
    for(i=8;i!=0;i--){
        if(byte & 0x80){
            i2c_tx_high_bit();
        }else{
            i2c_tx_low_bit();
        }
        byte = byte << 1;
    }
    i2c_read_ack();
}

//given any address and read/write mode (using the #define values from above) we send the address with the right mode bit
void i2c_tx_address(char addr, char mode){
    i2c_start();
    addr = (addr << 1) + mode;
    i2c_tx(addr);
}

//recieve the data from the slave device by setting the bit high or low in the data_in variable and shifting each bit recieved
char i2c_rx(void){
    char data_in = 0x00;
    P4DIR &= ~SDA; // SDA input port
    for(j=0; j<I2C_DELAY*2; j++){}

    for(i=8;i!=0;i--){
    data_in = data_in << 1;
        if(P4IN & SDA){
            data_in = data_in + 0x01;
        }
        i2c_clock_cycle();
    }
    for(j=0; j<I2C_DELAY; j++){}
    P4DIR |= SDA; // SDA output port
    return data_in;
}

//function to read the temperature value given a sensor address and address of the register we are trying to read. Returns the value.
char read_temp(char sensor, char addr){
    int data;
    i2c_tx_address(sensor, WRITE_MODE);
    i2c_tx(addr);
    i2c_tx_address(sensor, READ_MODE);
    data = i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//read accelerometer axis data and return the value read. Must call this three times to get each axis.
//takes in the address of the axis register data (see define statements above)
char read_accel(char addr){
    char data;
    i2c_tx_address(ACCEL_ADDR, WRITE_MODE);
    i2c_tx(addr);
    i2c_tx_address(ACCEL_ADDR, READ_MODE);
    data = i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//transmit to the pressure sensor, used inside the get pressure function
void i2c_tx_pressure(char addr){
    i2c_tx_address(PRESSURE_ADDR, WRITE_MODE);
    i2c_tx(addr);
    i2c_stop();
}

//Get the pressure coefficients, these coefficients are only received once and saved and then used for pressure calculation
void get_pressure_coeff(){
    for(k = 0; k < 8; k++){
        i2c_tx_pressure((0xA0+k*2));
        i2c_tx_address(PRESSURE_ADDR, READ_MODE);
        c[k] = i2c_rx();
        i2c_ack();
        c[k] = c[k] << 8;
        c[k] = c[k] + i2c_rx();
        i2c_nack();
        i2c_stop();
    }
}

//return the value read from the pressure sensor
unsigned long i2c_rx_pressure(){
    unsigned long data;
    i2c_tx_address(PRESSURE_ADDR, READ_MODE);
    data = i2c_rx();
    data = data << 8;
    i2c_ack();
    data = data + i2c_rx();
    data = data << 8;
    i2c_ack();
    data = data + i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//calculate the value returned from the pressure sensor using the necessary equations to get the actual pressure value and pressure temp value
void convert_pressure(){
    long temp_var;
    long long calc_pressure;

    dT = c[5];
    dT = dT << 8;
    dT = digital_press_temp - dT;
    press_temp = c[6] >> 2;
    temp_var = dT >> 4;
    press_temp = press_temp * temp_var;
    press_temp = press_temp >> 17;
    press_temp = press_temp + 2000;

    OFF = c[2];
    OFF = OFF << 17;
    temp_var = temp_var * c[4];
    temp_var = temp_var >> 2;
    OFF = OFF + temp_var;
    SENS = c[1];
    SENS = SENS << 16;
    temp_var = dT >> 4;
    temp_var = temp_var * c[3];
    temp_var = temp_var >> 3;
    SENS = SENS + temp_var;

    calc_pressure = SENS >> 8;
    temp_var = digital_press;
    temp_var = temp_var >> 8;
    calc_pressure = calc_pressure * temp_var;
    calc_pressure = calc_pressure >> 5;
    calc_pressure = calc_pressure - OFF;
    calc_pressure = calc_pressure >> 15;
    pressure = calc_pressure;
    for(i=0; i < 10; i++){}
}

//-----UART-----

//Send UART config messages (takes a configuration message in the form of a character array"
void UART_config_GPS_msgs(char disable[29]){
    for(i=0; i<29; i++){
        UCA0TXBUF = disable[i];
        //P6OUT ^= BIT1; //toggle LED
        for(j=0; j<150;j++){}
    }
}

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //------------------Configure I2C as I/O ports (bit banged)-----------------
    P6DIR |= BIT0; //  LED 1 on Port 6.0 as output
    P6OUT |= BIT0; //  LED off initially

    P4DIR |= SCL; // SCL output port
    P4DIR |= SDA; // SDA output port

    P4OUT |= SCL; // Set SCL high as default
    P4OUT |= SDA; // Set SDA high ad default

    //-------SPI-------
    P6DIR |= BIT2;  // LED Power-on
    P6OUT &= ~BIT2; //set LED on when powered

    UCA1CTLW0 = UCSWRST;        //  Placing UCA1CTLW0 into SW reset
    UCA1CTLW0 |= UCSSEL__SMCLK; //  Clock set to SMCLK 1MHz
    UCA1BRW = 10;               //  Divide down to 100kHz
    UCA1CTLW0 |= UCSYNC;        //  Synchronous Mode
    UCA1CTLW0 |= UCMST;         //  SPI Master
    UCA1CTLW0 |= UCMSB;         //  MSB first
                                //  Settings from eUSCI - SPI Mode Excerpt from SLAU208 Texas Instruments
    //  SPI PORTS
    P4SEL1 &= ~SCLK;            //  P4.1 SCLK
    P4SEL0 |= SCLK;
    P4SEL1 &= ~MOSI;            //  P4.3 SIMO
    P4SEL0 |= MOSI;
    P4SEL1 &= ~MISO;            //  P4.2 SOMI
    P4SEL0 |= MISO;
    P4DIR |= NSS;               //  P4.0 NSS/STE (Manually Toggled)

    UCA1CTLW0 &= ~UCSWRST;      //  SPI operation Ready

    P4OUT |= NSS;               //  Turn off Chip Select

    delay(1500);

    //-------UART--------
    P6DIR |= BIT1; // LED Power-on
    P6OUT |= BIT1; //set LED off when powered

    UCA0CTLW0 |= UCSWRST; //put A0 in software reset

    UCA0CTLW0 |= UCSSEL__ACLK; //Set clock

    //Clock dividing for 9600 baud rate on UART
    UCA0BRW = 3;
    UCA0MCTLW |= 0x9200;

    //Set the TX and RX channels to UART
    P1SEL1 &= ~BIT6;
    P1SEL0 |= BIT6;
    P1SEL1 &= ~BIT7;
    P1SEL0 |= BIT7;

    PM5CTL0 &= ~LOCKLPM5; // Turn on GPIO

    //for UART
    UCA0CTLW0 &= ~UCSWRST; //Take A0 out of software reset

    for(i=0;i<25;i++){
      delay(10000);
    }

    //set the LoRa Settings for TX
    SPI_tx(OPMODE_01, MODE_LORA_SLEEP); //Register Operation mode set to Sleep LoRa
    delay(100); // delay to ensure we are in sleep  mode
    SPI_tx(FIFO_TX_BASE_ADDR_0E, 0x00); //Set FIFO Tx Addr to 0x00
    SPI_tx(FIFO_RX_BASE_ADDR_0F, 0x00);
    SPI_tx(OPMODE_01, MODE_LORA_STBY); //Register Operation mode set to Standby LoRa
    SPI_tx(FR_MSB_06, 0xE4); //Set to 912 MHz MSB
    SPI_tx(FR_MID_07, 0x00); //Set to 912 MHz MidSB
    SPI_tx(FR_LSB_08, 0x00); //Set to 912 MHz LSB
    SPI_tx(POW_CONFIG_09, 0x8F); //Set Power Output
    SPI_tx(PREAMBLE_LEN_MSB_20, 0x00); //Set Preamble MSB
    SPI_tx(PREAMBLE_LEN_LSB_21, 0x08); //Set Preamble LSB
    SPI_tx(MODEM_CONFIG_1_1D, 0x72); //Modem Config 1 | 7 = bandwidth, 2 = coding rate + explicit header
    SPI_tx(MODEM_CONFIG_2_1E, 0xA4); //Modem Config 2 | A = Spreading factor, 0 = normal (single packet mode) + CRC enable (MUST BE ENABLED)
    SPI_tx(MODEM_CONFIG_3_26, 0x00); //Modem Config 3 | Low Data Rate Optimize, LNA
    SPI_tx(PAYLOAD_LEN_22, PAYLOAD_LEN); // Set Payload Len

    //-------Reset Temp Sensor------------

    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(0x2F); //Reset code
    i2c_stop();
    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(0x2F); //reset code
    i2c_stop();
    for(i=0; i<250; i++){}

    i2c_tx_address(INT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(TEMP_CONFIG);
    i2c_tx(0x80); //configure to 16-bit mode
    i2c_stop();
    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(TEMP_CONFIG);
    i2c_tx(0x80); //configure to 16-bit mode
    i2c_stop();

    //read in the configuration registers, uncomment for debugging
    //temp_config_int = read_temp(INT_TEMP_ADDR, TEMP_CONFIG);
    //temp_config_ext = read_temp(EXT_TEMP_ADDR, TEMP_CONFIG);

    //---------Config Accelerometer--------------
    i2c_tx_address(ACCEL_ADDR, WRITE_MODE);
    i2c_tx(0x20); //write to the control register
    i2c_tx(0x17); //take out of power down mode and set to 10Hz generation of signal
    i2c_stop();

    //--------Config Pressure-------------------
    i2c_tx_pressure(0x1E); //reset

    for(i=0; i<275; i++){} //3ms delay

    //get pressure coeffs
    get_pressure_coeff();

    for(i=0; i<254; i++){}

    //init all values to 0 so we don't accidentally send funky data
    LORA_PAYLOAD.LORA_accel_x = 0;
    LORA_PAYLOAD.LORA_accel_y = 0;
    LORA_PAYLOAD.LORA_accel_z = 0;
    LORA_PAYLOAD.LORA_ext_temp = 0;
    LORA_PAYLOAD.LORA_int_temp = 0;
    LORA_PAYLOAD.LORA_pressure = 0;

    //--------Send UART config messages--------
    for(k=0; k < 8; k++){
        UART_config_GPS_msgs(diable_GLL);
        UART_config_GPS_msgs(diable_GSV);
        UART_config_GPS_msgs(diable_GSA);
        UART_config_GPS_msgs(diable_VTG);
        UART_config_GPS_msgs(diable_RMC);
    }
    k = 0; //set k=0 to ensure manual incrementing of k for UART is setup correctly
    //fixes a bug where the array skips the first character, always a $
    GPS_GNGGA[0] = '$';
    //LORA_PAYLOAD.LORA_GPS[0] = '$'; //this line isn't necessary if using real data as it gets copied from "GPS_GNGGA"
    //DEBUG ONLY, COMMENT THIS FOR LOOP WHEN READY TO GET REAL UART DATA
    //for(i=1; i<95; i++){
        //LORA_PAYLOAD.LORA_GPS[i] = '!';
    //}
    //LORA_PAYLOAD.LORA_GPS[95] = '\n';

    UCA0IE &= ~UCRXIE; //disable UART receive interrupt (do this here so we don't recieve data too early)
    __enable_interrupt();

    delay(10000);

    while(1){

        //get temperatures
       int_temp = read_temp(INT_TEMP_ADDR, TEMP_MSB);
       int_temp = int_temp << 8;
       int_temp = int_temp + read_temp(INT_TEMP_ADDR, TEMP_LSB);
       LORA_PAYLOAD.LORA_int_temp = int_temp;
       ext_temp = read_temp(EXT_TEMP_ADDR, TEMP_MSB);
       ext_temp = ext_temp << 8;
       ext_temp = ext_temp + read_temp(EXT_TEMP_ADDR, TEMP_LSB);
       LORA_PAYLOAD.LORA_ext_temp = ext_temp;
       for(i=0; i<10; i++){}

       //get accelerometer
       accel_x = read_accel(ACCEL_X_MSB);
       accel_x = accel_x << 8;
       accel_x = accel_x + read_accel(ACCEL_X_LSB);
       LORA_PAYLOAD.LORA_accel_x = accel_x;
       accel_y = read_accel(ACCEL_Y_MSB);
       accel_y = accel_y << 8;
       accel_y = accel_y + read_accel(ACCEL_Y_LSB);
       LORA_PAYLOAD.LORA_accel_y = accel_y;
       accel_z = read_accel(ACCEL_Z_MSB);
       accel_z = accel_z << 8;
       accel_z = accel_z + read_accel(ACCEL_Z_LSB);
       LORA_PAYLOAD.LORA_accel_z = accel_z;
       for(i=0; i < 10; i++){}

       //get pressure
       i2c_tx_pressure(GET_PRESSURE);
       for(i=0; i<900; i++){} //9ms delay
       i2c_tx_pressure(READ_PRESS);
       digital_press = i2c_rx_pressure();

       i2c_tx_pressure(GET_PRESS_TEMP);
       for(i=0; i<900; i++){} //9ms delay
       i2c_tx_pressure(READ_PRESS);
       digital_press_temp = i2c_rx_pressure();

       convert_pressure();
       LORA_PAYLOAD.LORA_pressure = pressure;

       //set GPS data to struct value, if we need to debounce a value, this is how we would do it.
          /*for(k = 0; k < 100; k++){
              LORA_PAYLOAD.LORA_GPS[k] = temp[k]; //we set this rather than sending the GPS_GNGGA variable directly so we have a slight "debounce" from recieving it
              //UCA0TXBUF = LORA_PAYLOAD.LORA_GPS[k]; //uncomment this line to debug what is saving to the struct variable from the UART line
              if(temp[k] == '\n'){
                  k = 0;
              }
              //for(i=0; i<150;i++){} //this line was a delay for UART transmission
          }*/

       UCA0IE |= UCRXIE; //enable UART receive interrupt
       delay(10000);

       //Send the character array of all the data to LoRa! Yay! Sending MSB first
          SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //set FIFO pointer to 0x00

          //Write the radiohead header
          P4OUT &= ~NSS;
          UCA1TXBUF = 0x80;           // Write + FIFO Transmit register (1 + 0000000)
          delay(SPI_DELAY);
          UCA1TXBUF = 0xFC;           // Header to
          delay(SPI_DELAY);
          UCA1TXBUF = 0XFB;           // Header from
          delay(SPI_DELAY);
          UCA1TXBUF = 0x5B;           // Header ID
          delay(SPI_DELAY);
          UCA1TXBUF = 0x5B;           // Header Key
          delay(SPI_DELAY);
          UCA1TXBUF = 0x5B;           // Start of transmission character (deprecated)
          delay(SPI_DELAY);

          //transmit sensor data
          SPI_tx_two_bytes(LORA_PAYLOAD.LORA_int_temp);
          SPI_tx_two_bytes(LORA_PAYLOAD.LORA_ext_temp);
          SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_x);
          SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_y);
          SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_z);
          UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 24);
          delay(SPI_DELAY);
          UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 16);
          delay(SPI_DELAY);
          UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 8);
          delay(SPI_DELAY);
          UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure);
          delay(SPI_DELAY);
          UCA1TXBUF = (press_temp >> 24);
          delay(SPI_DELAY);
          UCA1TXBUF = (press_temp >> 16);
          delay(SPI_DELAY);
          UCA1TXBUF = (press_temp >> 8);
          delay(SPI_DELAY);
          UCA1TXBUF = (press_temp);
          delay(SPI_DELAY);
          //transmit GPS data
          for(i=0; i <100; i++){
              UCA1TXBUF = GPS_GNGGA[i];
              delay(SPI_DELAY);
          }
          P4OUT |= NSS; //SPI chip select

          SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa back to 0x00

          SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

          //long delay because we don't need to send that often and we wanna get the GPS data
          for(i=0;i<12;i++){
            delay(10000);
          }

    }
    //return 0; //never reached, left here if needed in the future
}

//UART receive interrupt
#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void){
    GPS_GNGGA[h] = UCA0RXBUF;
    P6OUT ^= BIT1; //toggle LED
    if(h==99){
        h=0;
        UCA0IE &= ~UCRXIE; //disable UART receive interrupt
    }else{
        h++;
    }
}
