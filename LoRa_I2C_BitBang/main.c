#include <msp430.h> 

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

//--------------Globals--------------------
char j, k;
char i2c_status = 0x00; // 0b00000000 -- | 0 | 0 | 0 | 0 || 0 | 0 | 0 | 1-NAK | (currently unused)
// char temp_config_int, temp_config_ext; //uncomment this line if uncommenting the lines that get the config of the temp sensors
int i;
int int_temp, ext_temp, accel_x, accel_y, accel_z = 0; //actual temperature (internal and external) and acceleration values on each axis
unsigned int c[8]; //coefficients for the pressure sensor
unsigned long digital_press, digital_press_temp = 0; //these are the variables that are used in calculating the pressure, not the actual pressure values
long dT, press_temp, pressure = 0; //actual temperature from pressure sensor and pressure
long long OFF, SENS = 0; //used for pressure calculation
//--------------Functions--------------------

//cycle the i2c clock line high then low
void i2c_clock_cycle(void){
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SCL; //SCL LOW
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

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	//------------------Configure I/O ports-----------------
    P4DIR |= SCL; // SCL output port
    P4DIR |= SDA; // SDA output port

    P4OUT |= SCL; //Set SCL high as default
    P4OUT |= SDA; //Set SDA high ad default

    P6DIR |= BIT2; // LED Power-on

    P6OUT &= ~BIT2; //set LED on when powered


    PM5CTL0 &= ~LOCKLPM5; // Turn on GPIO

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

    while(1){

        //get temperatures
       int_temp = read_temp(INT_TEMP_ADDR, TEMP_MSB);
       int_temp = int_temp << 8;
       int_temp = int_temp + read_temp(INT_TEMP_ADDR, TEMP_LSB);
       ext_temp = read_temp(EXT_TEMP_ADDR, TEMP_MSB);
       ext_temp = ext_temp << 8;
       ext_temp = ext_temp + read_temp(EXT_TEMP_ADDR, TEMP_LSB);
       for(i=0; i<10; i++){}

       //get accelerometer
       accel_x = read_accel(ACCEL_X_MSB);
       accel_x = accel_x << 8;
       accel_x = accel_x + read_accel(ACCEL_X_LSB);
       accel_y = read_accel(ACCEL_Y_MSB);
       accel_y = accel_y << 8;
       accel_y = accel_y + read_accel(ACCEL_Y_LSB);
       accel_z = read_accel(ACCEL_Z_MSB);
       accel_z = accel_z << 8;
       accel_z = accel_z + read_accel(ACCEL_Z_LSB);
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

       for(i=0; i<100; i++){} //delay so data has time to update
    }

	return 0;
}
