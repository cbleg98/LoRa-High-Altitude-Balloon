/* LoRa_Sample_Struct_Payload/main.c
 * This struct sample was created to demonstrate how to save/use the data from the board in a struct format
 * 
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 */

#include <msp430.h> 

int i;
//----PAYLOAD STRUCT TO SEND DATA TO LORA-----
struct payload{
    long LORA_pressure;
    int LORA_int_temp;
    int LORA_ext_temp;
    int LORA_accel_x;
    int LORA_accel_y;
    int LORA_accel_z;
    char LORA_GPS[100];
};

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    struct payload LORA_PAYLOAD; //THIS IS THE LORA PAYLOAD ALL MEMBERS ARE DAYA TO BE SENT TO THE LORA

    //init all values to 0 so we don't accidentally send funky data
    LORA_PAYLOAD.LORA_accel_x = 230; //230*0.061 = 14.03mg
    LORA_PAYLOAD.LORA_accel_y = -222; //-222*0.061 = -13.542mg
    LORA_PAYLOAD.LORA_accel_z = 16160; // 16160*0.061 = 985.76mg
    LORA_PAYLOAD.LORA_ext_temp = 0x0A95; //21.1640625 (see google doc or onedrive)
    LORA_PAYLOAD.LORA_int_temp = 0xFFEC; //-10.625 (see google doc or onedrive)
    LORA_PAYLOAD.LORA_pressure = 84832; //848.32 mbar
    LORA_PAYLOAD.LORA_GPS[0] = '$';
    for(i=1; i<95; i++){
        LORA_PAYLOAD.LORA_GPS[i] = '!';
    }
    LORA_PAYLOAD.LORA_GPS[95] = '\n';
    /* This simulates real data in the GPS data field. First is the $, then data (all ! in this case) then the newline character (\n)
     * then some blank spaces (because it doesn't always fill up the array all the way and the size can vary)
     *
     * EX: LORA_PAYLOAD.LORA_GPS = {'$','!','!',...,'!',...,'!','!','\n', '0x00', '0x00', '0x00', '0x00'}
     */
	while(1){
	    //PUT LORA CODE HERE
	    for(i=0;i<10;i++){}
	}

	return 0;
}
