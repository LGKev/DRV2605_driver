/*
 * DRV2605.c
 *
 *  Created on: May 11, 2018
 *      Author: Kevin Kuwata
 *		
 */

#include "DRV2605.h"
#include "msp.h"

/* Global */
volatile uint8_t bytesExpectedToReceive = 0;
volatile uint8_t indexRX = 0;
volatile uint8_t I2CReceived[10]; //just picked 10 but it should be as small as possible


void initDriver(void){



        UCB3CTLW0 = UCSWRST; //unlock

              // Master,    i2c,    smclk (3mhz),   WR
        UCB3CTLW0 |= UCMST | UCMODE_3 | UCSSEL__SMCLK | UCTR | UCSYNC; //uscyn always 1 bc spi or i2c only no uart.
    //the system clock is not running at 3mhz! or if it is the slave is clock stretching
    //remember i2c is 400khz max and 100khz standard

    //need to divide 3Mhz down to 100 khz...
        UCB3BR0 |= 30;

    //EN pin and Trigger Pin both GPIO
       P7SEL0 &= ~(BIT5 | BIT6);
       P7SEL1 &= ~(BIT5 | BIT6);

       //secondary!! mode for i2c table 6-77 msp datasheet
       P6SEL0 &= ~(BIT6 | BIT7);
       P6SEL1 |= BIT6 | BIT7;

       P7DIR |= BIT5 | BIT6; //outputs
       P7OUT |= BIT5; // enable! NOTE: remember to do this

       UCB3CTL0 &= ~UCSWRST; //lock

    //enable RX TX interrupts to process receieved bytes. ///change the transmit/reciever bits... for read/write correct?
    UCB3IFG = 0; //clear existing interrupts
   // UCB3IE |=  UCNACKIE | UCRXIE0 | UCTXIE; //INTERRUPTS FOR TIMOUT, NACK, RX (BYTE RECIEVED), BYTE COMPLETE TRANSMIT


    /* byte received collect the response, clear flag and collect next response in a global array
     * byte transmitted, depending on mode write single or write multiple continue to write the next thing?
     *      or change modes, change to RX mode?
     * */
    NVIC_EnableIRQ(EUSCIB3_IRQn);

}


uint8_t DRVSingleWrite(uint8_t registerToWrite, uint8_t valueToWrite){
    beginTransmission(DRV_DEFAULT_ADDRESS);
    UCB3CTL0 |= UCTR;
    UCB3TXBUF = valueToWrite;
    stopTransmission();
//interrupt will trigger and then we will do a stop condition.
    return 1;
}

void beginTransmission(uint8_t address){
    UCB3I2CSA = address;
    UCB3CTL0 |= UCTXSTT;
}


void stopTransmission(void){
    EUSCI_B3->CTLW0 |= UCTXSTP; //send stop command;
}

void writeRegister(uint8_t address, uint8_t reg, uint8_t value){
    EUSCI_B3->CTLW0 |= UCTR;
    beginTransmission(address);
    while(UCB3STATW & UCBBUSY); // wait while busy
    UCB3TXBUF = reg;
    while(UCB3STATW & UCBBUSY); // wait while busy
    UCB3TXBUF = value;
    stopTransmission();
}

void readRegister(uint8_t address, uint8_t reg, uint8_t numBytes){

    while(UCB3STAT & UCBBUSY); //wait if busy
       UCB3I2CSA = address;
       UCB3CTL0 |= UCTR | UCTXSTT;

    while(UCB3CTLW0 & UCTXSTT);
       UCB3TXBUF = reg;
    while(!(UCB3IFG & UCTXIFG0)); //wait until buf is transmitted

       /* read and repeat start */
       UCB3CTL0 &= ~UCTR;
       UCB3CTL0 |= UCTXSTT;

       while(UCB3CTLW0 & UCTXSTT);

       /* collect and send stop command */
       I2CReceived[0] = UCB3RXBUF;
       UCB3CTL0 |= UCTXSTP;
    //beginTransmission(DRV_DEFAULT_ADDRESS);


  //  stopTransmission();



  //  UCB3I2CSA = address;
   // UCB3CTL0 |= UCTXSTT;

    /*
    UCB3CTL0 |= UCTR;
    UCB3TXBUF = reg;
    UCB3CTLW0 |= UCTXSTP;


    UCB3CTL0 |= UCTXSTT; /* repeated start */

    //UCB3CTL0 &=~UCTR; /* read mode */
    //UCB3I2CSA = address;

    bytesExpectedToReceive = numBytes;
    indexRX = 0; /*global variable */
}


/* =====================================================================================================================================================*/
/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/
/* ISR */
/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/
void EUSCIB3_IRQHandler(void)
{

    if (UCB3IFG & UCRXIFG0)
    {
        //clear flag
        UCB3IFG &= ~UCRXIFG0;
        //complete byte received, collect into global
        if(indexRX < bytesExpectedToReceive){
        I2CReceived[indexRX] = UCB3RXBUF; /* Global array to collect i2c data that the slave gave back.  */
        indexRX++; /* global and will be set to 0 when we set bytesExpectedToRecieve */
        }
        stopTransmission();
        //slave may send more bytes than expected, let them fall off.
    }
    else if(UCB3IFG & UCTXIFG0){
        UCB3IFG &= ~UCTXIFG0;
       // UCB3CTL0 |= UCTXSTP;
        //TX buffer empty
    }
    else if(UCB3IFG & UCNACKIFG){
        UCB3IFG &= ~UCNACKIFG;
       // UCB3CTL0 |= UCTXSTP;

        // nack received
    }
}

/*
 *
 * https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/589712?MSP430FR5969-Read-multiple-bytes-of-data-i2c-with-repeated-start-and-without-interrupts
 * oid i2c_init(){

    P1SEL1 |= BIT6 | BIT7;                  // configure I2C pins
    P1SEL0 &= ~(BIT6 | BIT6);               // configure I2C pins

    // I2C default uses SMCLK
    UCB0CTL1 |= UCSWRST;                    // put eUSCI_B in reset state
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C, master, sync
    UCB0BRW = 0x000A;                       // baud rate = SMCLK / 10 = 100khz
    UCB0CTL1 &= ~UCSWRST;                   // eUSCI_B in operational state
}

void i2c_write(uint8_t slv_addr, uint8_t reg_addr, uint8_t data){

    while(UCB0STAT & UCBBUSY);

    UCB0I2CSA = slv_addr;                   // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;            // transmitter mode and START condition.

    while (UCB0CTLW0 & UCTXSTT);
    UCB0TXBUF = reg_addr;
    while(!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF = data;
    while(!(UCB0IFG & UCTXIFG0));

    UCB0CTLW0 |= UCTXSTP;
    while(UCB0CTLW0 & UCTXSTP);             // wait for stop
}

uint8_t i2c_read(uint8_t slv_addr, uint8_t reg_addr){

    uint8_t data = 0;

    while(UCB0STAT & UCBBUSY);
    UCB0I2CSA = slv_addr;                   // set slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;            // transmitter mode and START condition.

    while(UCB0CTLW0 & UCTXSTT);
    UCB0TXBUF = reg_addr;
    while(!(UCB0IFG & UCTXIFG0));

    UCB0CTLW0 &= ~UCTR;                     // receiver mode
    UCB0CTLW0 |= UCTXSTT;                   // START condition

    while(UCB0CTLW0 & UCTXSTT);             // make sure start has been cleared
    UCB0CTLW0 |= UCTXSTP;                   // STOP condition
    while(!(UCB0IFG & UCRXIFG0));
    data = UCB0RXBUF;

    while(UCB0CTLW0 & UCTXSTP);

    return data;
}

multiple


void i2c_read_multi(uint8_t slv_addr, uint8_t reg_addr, uint8_t l, uint8_t *arr){

   uint8_t i;

   while(UCB0STAT & UCBBUSY);

   UCB0I2CSA = slv_addr;                   // set slave address

   UCB0CTLW0 |= UCTR | UCTXSTT;            // transmitter mode and START condition.

   while(UCB0CTLW0 & UCTXSTT);

   UCB0TXBUF = reg_addr;

   while(!(UCB0IFG & UCTXIFG0));

   UCB0CTLW0 &= ~UCTR;                     // receiver mode

   UCB0CTLW0 |= UCTXSTT;                   // START condition

   while(UCB0CTLW0 & UCTXSTT);             // make sure start has been cleared

   for (i = 0; i < l; i++) {

       while(!(UCB0IFG & UCRXIFG0));

       if(i == l - 1){

           UCB0CTLW0 |= UCTXSTP;           // STOP condition

       }

       arr[i] = UCB0RXBUF;

   }

   while(UCB0CTLW0 & UCTXSTP);

}
 * */
