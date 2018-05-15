#include "msp.h"
#include "DRV2605.h"

/**
 * main.c
 *
 * Author: Kevin Kuwata
 * Date: 1/14/18
 *
 * @brief: This is an example project to get the motor driver working. I am using the pinouts found
 * in the smartwatch v2 schematic.
 * Depends on I2C to communicate to the DRV2605 motor driver.
 * Also uses Enable and PWM pins.
 *          Enable pin           P7.5     GPIO
 *          PWM pin              P7.6     TA1.2   Primary           //trigger or pwm pin
 *          SDA                  P6.6     Tertiary// i bet you this is that bull shit needs to be secondary.
 *          SCL                  P6.7     Tertiary
 *
 *    LRA (disk motor) focused. Only can use Library 6 or Library 7 for preprogrammed
 *
 *    I2C UCB3
 *
 *
 */




/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/
/* ======================================================================================================================================================*/

void initHBLed(void){
    P9SEL0 &= ~BIT3;
    P9SEL1 &= ~BIT3;
    P9DIR |= BIT3;
    P9OUT |= BIT3;
}

void beatHeart(uint32_t duration){

    P9OUT|=BIT3;
    uint32_t duration_mS = duration*300; // .1 seconds, like 1 is .1 seconds, 10 is 1 second
    uint32_t i = 0;
    for(i=0; i<duration_mS; i++);
    P9OUT &= ~BIT3;
    for(i=0; i<duration_mS; i++);

}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	__enable_interrupt(); //globals

	initHBLed();
	initDriver();

	uint32_t i;
    P7OUT |= BIT5;



	while(1){
	    //beatHeart(50);

	//   DRVSingleWrite(0x01, 0xBB);
	    //uint8_t i = 0;
	    for(i=0; i<256; i++){
	        DRVSingleWrite(0x01, BIT7 );
	    }

	    /*

	    UCB3I2CSA = DRV_DEFAULT_ADDRESS;
	    UCB3TXBUF |= 0xAA;
	    UCB3CTLW0 |= UCTXSTT;

	    for(i = 0; i< 20000; i++);
	*/
	}


}

