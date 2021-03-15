/* LoRa_Board_Serial_Print/main.c
 * This firmware prints the board data to serial for debugging
 * 
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 */

#include <msp430.h> 

int i, k;
//----PAYLOAD STRUCT TO SEND DATA TO LORA-----
char gps_dat[100] = {'$','G','N','G','G','A',',','1','9','4','5','1','8','.','0','0',',','4','5','3','9','.','7','5','4','5','3',',','N',',','1','1','1','0','2','.','6','9','8','9','4',',','W',',','1',',','1','2',',','0','.','7','8',',','1','5','0','9','.','6',',','M',',','-','1','7','.','6',',','M',',',',','*','4','8','\r','\n'};
struct payload{
    long LORA_pressure;
    int LORA_int_temp;
    int LORA_ext_temp;
    int LORA_accel_x;
    int LORA_accel_y;
    int LORA_accel_z;
    char LORA_GPS[100];
};

void UART_tx_two_bytes(int val){
    UCA1TXBUF = (val >> 8);
    for(i=0; i<150;i++){}
    UCA1TXBUF = (val & 0x00FF);
    for(i=0; i<150;i++){}
}

void UART_tx_comma(void){
    UCA1TXBUF = ',';
    for(i=0; i<150;i++){}
}

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    UCA1CTLW0 |= UCSWRST; //put A0 in software reset
    UCA1CTLW0 |= UCSSEL__ACLK; //Set clock

        //Clock dividing for 9600 baud rate on UART
    UCA1BRW = 3;
    UCA1MCTLW |= 0x9200;

    //Set the TX and RX channels to UART
    P4SEL1 &= ~BIT3;
    P4SEL0 |= BIT3;

    PM5CTL0 &= ~LOCKLPM5;

    UCA1CTLW0 &= ~UCSWRST; //Take A0 out of software reset

    struct payload LORA_PAYLOAD; //THIS IS THE LORA PAYLOAD ALL MEMBERS ARE DAYA TO BE SENT TO THE LORA

    //init all values to 0 so we don't accidentally send funky data
    LORA_PAYLOAD.LORA_accel_x = 230; //230*0.061 = 14.03mg
    LORA_PAYLOAD.LORA_accel_y = -222; //-222*0.061 = -13.542mg
    LORA_PAYLOAD.LORA_accel_z = 16160; // 16160*0.061 = 985.76mg
    LORA_PAYLOAD.LORA_ext_temp = 0x0A95; //21.1640625 (see google doc or onedrive) //0x0A95
    LORA_PAYLOAD.LORA_int_temp = 0xFAB0; //-10.625 (see google doc or onedrive)
    LORA_PAYLOAD.LORA_pressure = 84832; //848.32 mbar
    for(i = 0; i < 100; i++){
        LORA_PAYLOAD.LORA_GPS[i] = gps_dat[i];
        if(gps_dat[i] == '\n'){
            i = 100;
        }
    }

    /* This simulates real data in the GPS data field. First is the $, then data (all ! in this case) then the newline character (\n)
     * then some blank spaces (because it doesn't always fill up the array all the way and the size can vary)
     *
     * EX: LORA_PAYLOAD.LORA_GPS = {'$','!','!',...,'!',...,'!','!','\n', '0x00', '0x00', '0x00', '0x00'}
     */
    while(1){

        //Example of how to send the character array
        UCA1TXBUF = '[';
        for(i=0; i<150;i++){}
        UART_tx_two_bytes(LORA_PAYLOAD.LORA_int_temp);
        //UART_tx_comma();
        UART_tx_two_bytes(LORA_PAYLOAD.LORA_ext_temp);
        //UART_tx_comma();
        UART_tx_two_bytes(LORA_PAYLOAD.LORA_accel_x);
        //UART_tx_comma();
        UART_tx_two_bytes(LORA_PAYLOAD.LORA_accel_y);
        //UART_tx_comma();
        UART_tx_two_bytes(LORA_PAYLOAD.LORA_accel_z);
        //UART_tx_comma();
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 24);
        for(i=0; i<150;i++){}
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 16) & 0x00FF;
        for(i=0; i<150;i++){}
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 8) & 0x00FF;
        for(i=0; i<150;i++){}
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure & 0x00FF);
        for(i=0; i<150;i++){}
        //UART_tx_comma();
        for(k = 0; k < 100; k++){
            UCA1TXBUF = LORA_PAYLOAD.LORA_GPS[k]; //uncomment this line to debug what is saving to the struct variable from the UART line
            //if(LORA_PAYLOAD.LORA_GPS[k] == '\n'){
                //k = 100;
            //}
            for(i=0; i<150;i++){}
        }
        UCA1TXBUF = ']';
        for(i=0; i<150;i++){}
        UCA1TXBUF = '\n';
        for(i=0; i<150;i++){}
        UCA1TXBUF = '\r';
        for(i=0; i<250;i++){} //delay so we aren't sending too often (can remove if other code delays enough already)

        LORA_PAYLOAD.LORA_ext_temp = rand();
        LORA_PAYLOAD.LORA_int_temp = rand();
        LORA_PAYLOAD.LORA_accel_x = rand();
        LORA_PAYLOAD.LORA_accel_y = rand();
        LORA_PAYLOAD.LORA_accel_z = rand();

    }

    return 0;
}
