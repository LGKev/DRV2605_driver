/*
 * DRV2605.c
 *
 *  Created on: May 11, 2018
 *      Author: Kevin Kuwata
 *		
 */

#include "DRV2605.h"
#include "msp.h"



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

    //set Slave Address
  //  UCB3I2CSA = DRV_DEFAULT_ADDRESS;





    //enable RX TX interrupts to process receieved bytes. ///change the transmit/reciever bits... for read/write correct?
    UCB3IFG = 0; //clear existing interrupts
    UCB3IE |=  UCNACKIE | UCRXIE0 | UCTXIE; //INTERRUPTS FOR TIMOUT, NACK, RX (BYTE RECIEVED), BYTE COMPLETE TRANSMIT




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
    stop_Transmission(DRV_DEFAULT_ADDRESS);


//interrupt will trigger and then we will do a stop condition.

    return 1;
}

void beginTransmission(uint8_t address){
    UCB3I2CSA = address;
    UCB3CTL0 |= UCTXSTT;
}


void stop_Transmission(uint8_t _address){
    EUSCI_B3->CTLW0 |= UCTXSTP; //send stop command;
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

