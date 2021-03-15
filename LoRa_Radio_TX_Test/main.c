/* LoRa_Radio_TX_Test/main.c
 * Sends test data to the ground station in the correct format from the LoRa RF95 radio.
 * 
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 * @author (SPI methods and setup) Larson Brandstetter
 */

#include <msp430.h> 
#include "reg_map.h"

#define PAYLOAD_LEN 0x21

//-----globals-----
//char registerStats[26];
char transmitted;
unsigned int i;

char txpack[20];

void delay(unsigned int j){
    unsigned int k;
    for(k = 0; k < j; k++){
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

    PM5CTL0 &= ~LOCKLPM5;       //  GPIO on

    P4OUT |= NSS;               //  Turn off Chip Select

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
            UCA1TXBUF = 0x48;
            delay(20);
            UCA1TXBUF = 0x49;
            delay(20);
            UCA1TXBUF = 0x48;
            delay(20);
            UCA1TXBUF = 0x49;
            delay(20);
            UCA1TXBUF = 0x48;
            delay(20);
            UCA1TXBUF = 0x49;
            delay(20);
            SPI_burst_tx(txpack);
            UCA1TXBUF = 0x00; // we need this to end the transmission for RadioHead? I dont think so
            delay(20);
            P4OUT |= NSS;

            SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa

            SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

            for(i=0;i<50;i++){
              delay(10000);
            }

        }

}
