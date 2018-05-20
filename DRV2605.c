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
       P7OUT &= ~BIT6; // connect to ground for INT

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


void autoCalibrationLRA(void){
    /* put into auto calibration mode: register 0x01 -> 0x07 */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, AUTO_CALIBRATION_MODE);

    /* Enter standby mode */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, STANDBY);

    /* SET RATED voltage 3v for motor refer to the setup guide. */
    writeRegister(DRV_DEFAULT_ADDRESS, RATED_VOLTAGE_R, 0x60);

    /* Set Over drive voltage */
    writeRegister(DRV_DEFAULT_ADDRESS, OVERDRIVE_CLAMP_V_R, 0x96);

    /* Fill in auto calibration registers  */
    writeRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R, 0xB6);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_1_R, 0x13);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_2_R, 0xF5);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_3_R, 0x80);


    /* library selection for ROM effects, library 6 for LRA */
    writeRegister(DRV_DEFAULT_ADDRESS, LIBRARY_R, 0x06);

    /* come out of standby mode  */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, 0x00);

    /* send go bit */
    writeRegister(DRV_DEFAULT_ADDRESS, GO_R, 1);

    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, STANDBY);

}

void autoCalibrationERM(void){
    /* put into auto calibration mode: register 0x01 pre config then auto calibrate  */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, 0x00);

    /* SET RATED voltage 3v for motor refer to the setup guide. */
    writeRegister(DRV_DEFAULT_ADDRESS, RATED_VOLTAGE_R, 0x90);

    /* Set Over drive voltage */
    writeRegister(DRV_DEFAULT_ADDRESS, OVERDRIVE_CLAMP_V_R, 0x96);

    /* Fill in auto calibration registers  */
    writeRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R, 0x36);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_1_R, 0x93);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_2_R, 0xF5);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_3_R, 0x80);


    /* library selection for ROM effects, library 6 for LRA */
    writeRegister(DRV_DEFAULT_ADDRESS, LIBRARY_R, 0x01);

    /* come out of standby mode  */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, 0x07);

    /* Fill in auto calibration registers   */
       writeRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R, 0x36);
       writeRegister(DRV_DEFAULT_ADDRESS, CTRL_1_R, 0x13);
       writeRegister(DRV_DEFAULT_ADDRESS, CTRL_2_R, 0xF5);
       writeRegister(DRV_DEFAULT_ADDRESS, CTRL_3_R, 0xA0);
       writeRegister(DRV_DEFAULT_ADDRESS, CTRL_4_R, 0x20);

    /* send go bit */
    writeRegister(DRV_DEFAULT_ADDRESS, GO_R, 1);

    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, STANDBY);
}


void preAutoCalibrationLRA(void){
    /* set up the feedback control register (3a) - (3c)  */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, 0x07);


    uint8_t valueToWrite = 0xB0 | 0x06; // equal to 0b1 011 01 01
    writeRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R , valueToWrite);

    /* set up rated voltage control register  NOTE: CLOSED_LOOP Only (3d)  */
    valueToWrite = 0x3E; // default value, but what does this even correspond to in volts?
    /* I don't think it actually is a voltage, but more of a scaling factor used in Equation 5.
     * I don't know the frequency of the LRA and I have not set the sample time. how do I find
     * the resonance frequency?
     * TODO: f_LRA needs to be determined, recalculate this equation 5*/
    writeRegister(DRV_DEFAULT_ADDRESS, RATED_VOLTAGE_R , valueToWrite);

    /* set up  OVER DRIVE CLAMP control register (3e) */
    valueToWrite = 0x89; // TODO: use the default value until I find what f_LRA is
    writeRegister(DRV_DEFAULT_ADDRESS, OVERDRIVE_CLAMP_V_R , valueToWrite); /* Responsible for max voltage in open loop, but allows for overshoot in closed loop */
    /* default values for voltage and OD, with 200 hz resonant is 2.97 v max so it should be fine. unless resonant goes down. */

    /* set up auto calibration time (500ms) , ZC_detect Time (100uS) Control Register 4  (3f) & (3k) */
    valueToWrite = 0x20; // default values, 00 10 0000
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_4_R , valueToWrite);

    /* set up Drive Time Control Register 1 (3g)  */
    valueToWrite = 0x93; //default values TODO: find F_LRA then get period then recalculate this.
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_1_R, valueToWrite);

    /* set up Sample_Time (200uS), Blanking Time (50uS), IDISS_time (50uS),  Control Register 2 (3h) - (3j)  */
    valueToWrite = 0xDA; //0b11 01 10 10 default values;
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_2_R , valueToWrite);

    /* Set the Go bit in 0x0C and then check the 4 registers:
     *  - bemf_gain (register 0x1A) bits 1 and 0
     *  - A_cal_comp (register  0x18)
     *  - a_cal_bemf (0x19)
     *  - diag_result == sucessful  calibration? */

    writeRegister(DRV_DEFAULT_ADDRESS, GO_R, 1);

}


void setGoBit(void){
    writeRegister(DRV_DEFAULT_ADDRESS, GO_R, 1);
    while(UCB3STATW & UCBBUSY);
}



void analogMode(void){
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, MODE_ANALOG_INPUT);
    writeRegister(DRV_DEFAULT_ADDRESS, CTRL_3_R, 0x81);
}

void setAndPlay(uint8_t waveformID){
    /* come out of standby */
    writeRegister(DRV_DEFAULT_ADDRESS, MODE_R, 0x00); //internal trigger

    writeRegister(DRV_DEFAULT_ADDRESS, 0x04, waveformID);

    writeRegister(DRV_DEFAULT_ADDRESS, GO_R, 1);
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


/* need to do a read modify write. */
void writeRegister(uint8_t address, uint8_t reg, uint8_t value){

    //while(UCB3STATW & UCBBUSY); // wait while busy
    while(UCB3CTLW0 & UCTXSTT); //soon as this is no longer true send the next thing
    UCB3I2CSA = address;
    UCB3CTL0 |= UCTR | UCTXSTT;

    while(UCB3CTLW0 & UCTXSTT); //soon as this is no longer true send the next thing
    UCB3TXBUF = reg;
    while(!(UCB3IFG & UCTXIFG0)); //when empty send next thing

    UCB3TXBUF = value;
    while(!(UCB3IFG & UCTXIFG0)); //when empty send next thing
    UCB3CTL0 |= UCTXSTP;
}

uint8_t readRegister(uint8_t address, uint8_t reg){

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
     //  while(!(UCB3IFG & UCRXIFG0));
       UCB3CTL0 |= UCTXSTP;
       /* collect the bytes requested, stuff into array, then send stop*/
       uint8_t rxValue = UCB3RXBUF;
       while(UCB3CTLW0 & UCTXSTP); //clear after stop been set and come out.

       return rxValue;
}


/* Auto Calibration for LRA page 26 */
/*  1) set to calibration mode : write to register 0x01 with 0x07
 *  2) set the following registers:
 *      0x1A:
 *           bit7 = n_LRA
 *           bit6 - bit4 = brake factor
 *           bit3 - bit2 = loop gain
 *           bit1 - bit0 = back emf gain
 *      0x1B :
 *          rated voltage
 *          od_clamp
 *          auto_calibration_time
 *          drive time
 *          sample time
 *          blanking time
 *          idiss_time
 *          zc_Detect_time
 *
 *          then set go bit in 0x0c
 *
 *          check status register to see if any faults??! what short?
 *
 *
 *
 *
 *
 * */

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

