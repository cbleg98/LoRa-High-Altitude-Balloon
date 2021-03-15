/* LoRa_GPS_Only_Test/main.c
 * This code is the test bed for sending the data to the
 * featherboard LoRa radio ground station
 * 
 * Version 2.0
 *
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 */


#include <msp430.h> 
#include "reg_map.h"

#define PAYLOAD_LEN 0x80

//-----globals-----
//---LoRa
//char registerStats[26];
char transmitted;
unsigned int i;
char txpack[200];
//---GPS
char j, k = 0;
// The below character arrays are the strings needing to be sent to the GPS to properly configure the messages
char diable_GLL[] = {'$','P','U','B','X',',','4','0',',','G','L','L',',','0',',','0',',','0',',','0',',','0',',','0','*','5','C','\r','\n'};
char diable_GSV[] = {'$','P','U','B','X',',','4','0',',','G','S','V',',','0',',','0',',','0',',','0',',','0',',','0','*','5','9','\r','\n'};
char diable_GSA[] = {'$','P','U','B','X',',','4','0',',','G','S','A',',','0',',','0',',','0',',','0',',','0',',','0','*','4','E','\r','\n'};
char diable_VTG[] = {'$','P','U','B','X',',','4','0',',','V','T','G',',','0',',','0',',','0',',','0',',','0',',','0','*','5','E','\r','\n'};
char diable_RMC[] = {'$','P','U','B','X',',','4','0',',','R','M','C',',','0',',','0',',','0',',','0',',','0',',','0','*','4','7','\r','\n'};
char GPS_GNGGA[100];
char LORA_GPS[100];
char temp[100];
//char temp2 = 'x';

//gps
//Send UART config messages (takes a configuration message in the form of a character array"
void UART_config_GPS_msgs(char disable[29]){
    for(i=0; i<29; i++){
        UCA0TXBUF = disable[i];
        for(j=0; j<150;j++){}
    }
}

//lora
void delay(unsigned int l){
    unsigned int k;
    for(k = 0; k < l; k++){
        P6OUT ^= BIT2;              //  Toggle LED3
    }
}

void SPI_tx(char addr, char data){
    char temp;
    temp = 0x80 + addr; //0x80 is the send bit for SPI
    P4OUT &= ~NSS;
    UCA1TXBUF = temp;
    UCA1TXBUF = data;
    delay(20);
    P4OUT |= NSS;
}

void SPI_burst_tx(char data[20]){
    for(i=0; i < PAYLOAD_LEN; i=i+2){
        UCA1TXBUF = data[i];
        delay(20);
        UCA1TXBUF = data[i+1];
        delay(20);
    }
    delay(PAYLOAD_LEN*10);
}

char SPI_read_reg(char addr){
    char data;
    P4OUT &= ~NSS;
    UCA1TXBUF = addr;
    UCA1TXBUF = addr;
    data = UCA1RXBUF;
    delay(20);
    P4OUT |= NSS;
    return data;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

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

    //  OTHER PORTS
    P6DIR |= BIT0;              //  LED 1 on Port 6.0 as output
    P6OUT |= BIT0;              //  LED off initially

    P6DIR |= BIT1;              //  LED 2 on Port 6.1 as output
    P6OUT |= BIT1;              //  LED off initially

    P6DIR |= BIT2;              //  LED 3 on Port 6.2 as output
    P6OUT |= BIT2;              //  LED off initially

    UCA1CTLW0 &= ~UCSWRST;      //  SPI operation Ready


    //GPS
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

    PM5CTL0 &= ~LOCKLPM5;       //  GPIO on

    P4OUT |= NSS;               //  Turn off Chip Select

    //gps
    UCA0CTLW0 &= ~UCSWRST; //Take A0 out of software reset


    //Send UART config messages
    for(k=0; k < 6; k++){
        UART_config_GPS_msgs(diable_GLL);
        UART_config_GPS_msgs(diable_GSV);
        UART_config_GPS_msgs(diable_GSA);
        UART_config_GPS_msgs(diable_VTG);
        UART_config_GPS_msgs(diable_RMC);
    }

    k = 0;
    GPS_GNGGA[0] = '$';

    UCA0IE &= ~UCRXIE; //enable UART receive interrupt
    __enable_interrupt();

    delay(10000);

    for(i=0;i<20;i=i+2){
        txpack[i]=0x48;
        txpack[i+1]=0x49;
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

        /*unsigned int j,k; //Reading first 26 registers that we set
        k = 0;
        for(j = 0x01; j < 0x27; j++){
            P4OUT &= ~NSS;
            UCA1TXBUF = j;
            UCA1TXBUF = 0x00;
            registerStats[k] = UCA1RXBUF;
            delay(20);
            P4OUT |= NSS;
            k++;
            delay(5);
        }*/

        while(1){

            //set GPS data to struct value
               for(k = 0; k < 100; k++){
                   LORA_GPS[k] = GPS_GNGGA[k]; //we set this rather than sending the GPS_GNGGA variable directly so we have a slight "debounce" from recieving it
                   //UCA0TXBUF = LORA_PAYLOAD.LORA_GPS[k]; //uncomment this line to debug what is saving to the struct variable from the UART line
                   if(GPS_GNGGA[k] == '\n'){
                       k = 0;
                   }
                   //for(i=0; i<150;i++){} //this line was a delay for UART transmission
               }

               UCA0IE |= UCRXIE; //enable UART receive interrupt
               delay(10000);

            SPI_tx(FIFO_ADDR_PTR_0D, 0x00);

            P4OUT &= ~NSS;
            UCA1TXBUF = 0x80;           // Write + FIFO Transmit register (1 + 0000000)
            delay(20);
            UCA1TXBUF = 0xFC;           //  Header to
            delay(20);
            UCA1TXBUF = 0XFB;           //  Header from
            delay(20);
            UCA1TXBUF = 0x5B;
            delay(20);
            UCA1TXBUF = 0x5B;
            delay(20);
            UCA1TXBUF = 0x5B;
            delay(20);

            for(i=0; i <100; i++){
                UCA1TXBUF = temp[i];
                delay(20);
            }

            /*for(k = 0; k < 100; k++){
                UCA1TXBUF = LORA_GPS[k];
                delay(20);
                if(LORA_GPS[k] == '\n'){
                    UCA1TXBUF = ']';
                    delay(20);
                    k = 100;
                }
                else if(k == 99){
                    UCA1TXBUF = ']';
                    delay(20);
                }
            }*/
            //SPI_burst_tx(txpack);
            //UCA1TXBUF = 0x00; // we need this to end the transmission for RadioHead? I dont think so
            //delay(20);
            P4OUT |= NSS;

            SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa

            SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

            //for(i=0;i<200;i++){
                //txpack[i] = UCA0RXBUF;
            //}

            for(i=0;i<50;i++){
              delay(10000);
            }

        }

}

#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void){
    temp[j] = UCA0RXBUF;
    //temp2 = UCA0RXBUF;
//    if(GPS_GNGGA[j] == '\n' || j == 99){ //get packets divided by new line, or prevent overflow
//        j = 0;
        P6OUT ^= BIT1; //toggle LED
//    }else{
    if(j==99){
        j=0;
        UCA0IE &= ~UCRXIE; //enable UART receive interrupt
    }else{
        j++;
    }
//    }
}
