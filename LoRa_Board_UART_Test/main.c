/* LoRa_Board_UART_Test/main.c
 * Configures the GPS and checks data being sent, option to set variable to see contents
 * 
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 */

#include <msp430.h> 

char i;
char j, k = 0;
// The below character arrays are the strings needing to be sent to the GPS to properly configure the messages
char diable_GLL[] = {'$','P','U','B','X',',','4','0',',','G','L','L',',','0',',','0',',','0',',','0',',','0',',','0','*','5','C','\r','\n'};
char diable_GSV[] = {'$','P','U','B','X',',','4','0',',','G','S','V',',','0',',','0',',','0',',','0',',','0',',','0','*','5','9','\r','\n'};
char diable_GSA[] = {'$','P','U','B','X',',','4','0',',','G','S','A',',','0',',','0',',','0',',','0',',','0',',','0','*','4','E','\r','\n'};
char diable_VTG[] = {'$','P','U','B','X',',','4','0',',','V','T','G',',','0',',','0',',','0',',','0',',','0',',','0','*','5','E','\r','\n'};
char diable_RMC[] = {'$','P','U','B','X',',','4','0',',','R','M','C',',','0',',','0',',','0',',','0',',','0',',','0','*','4','7','\r','\n'};
char GPS_GNGGA[100];

//Send UART config messages (takes a configuration message in the form of a character array"
void UART_config_GPS_msgs(char disable[29]){
    for(i=0; i<29; i++){
        UCA0TXBUF = disable[i];
        for(j=0; j<150;j++){}
    }
}

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    P6DIR |= BIT1; // LED Power-on
    P6OUT &= ~BIT1; //set LED on when powered

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

	PM5CTL0 &= ~LOCKLPM5;

	UCA0CTLW0 &= ~UCSWRST; //Take A0 out of software reset

	UCA0IE |= UCRXIE; //enable UART receive interrupt
	__enable_interrupt();

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

	while(1){
	    //Example of how to send the character array
	    for(k = 0; k < 100; k++){
	        //UCA0TXBUF = GPS_GNGGA[k]; //Turn UCA0TXBUF to whatever variable you want to store the data in
	        if(GPS_GNGGA[k] == '\n'){
	            k = 0;
	        }
	        for(i=0; i<150;i++){} //this line was a delay for UART transmission, you may be able to remove if using a different protocol (i.e. I2C, SPI)
	    }
	    for(i=0; i<250;i++){} //delay so we aren't sending too often (can remove if other code delays enough already)


	}

	return 0;
}

#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void){
    UCA0TXBUF = UCA0RXBUF;
    //GPS_GNGGA[j] = UCA0RXBUF;
    if(GPS_GNGGA[j] == '\n' || j == 99){ //get packets divided by new line, or prevent overflow
        j = 0;
        P6OUT ^= BIT1; //toggle LED
    }else{
        j++;
    }
}
