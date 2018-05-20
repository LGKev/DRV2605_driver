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


extern uint8_t I2CReceived[10]; //just picked 10 but it should be as small as possible

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
	//analogMode();

 //   beatHeart(3000);


	/* device reset 7th bit to 0x01 */
	//writeRegister(DRV_DEFAULT_ADDRESS, 0x01, 0x80); //7th bit in binary


	/* set up for auto calibration */
	//preAutoCalibrationLRA();

	//autoCalibrationLRA();
	autoCalibrationERM();
	//setAndPlay(118);

	beatHeart(500);
	//LRA resonance is 127??

	// comp result: 12 decimal, 0x0C
	// auto bemf: 108

#ifdef fixed
   uint8_t a = readRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R); //0x1A
   uint8_t aa = readRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R); //0x1A

   uint8_t b = readRegister(DRV_DEFAULT_ADDRESS, STATUS_R); //0x00
   uint8_t bb = readRegister(DRV_DEFAULT_ADDRESS, STATUS_R); //0x00

   uint8_t c = readRegister(DRV_DEFAULT_ADDRESS, LRA_RESONANCE_R); //0x22
   uint8_t cc = readRegister(DRV_DEFAULT_ADDRESS, LRA_RESONANCE_R); //0x22

   uint8_t d =   readRegister(DRV_DEFAULT_ADDRESS, AUTO_CAL_COMP_RESULT_R); //0x18
   uint8_t dd =   readRegister(DRV_DEFAULT_ADDRESS, AUTO_CAL_COMP_RESULT_R); //0x18

   uint8_t e =   readRegister(DRV_DEFAULT_ADDRESS, AUTO_CAL_BEMF_RESULT_R); //0x19
   uint8_t ee =   readRegister(DRV_DEFAULT_ADDRESS, AUTO_CAL_BEMF_RESULT_R); //0x19
   uint8_t f = readRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R); //0x1A
   uint8_t ff = readRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R); //0x1A
   uint8_t fff = readRegister(DRV_DEFAULT_ADDRESS, FB_CTRL_R); //0x1A


#endif




	while(1){
      //  P7OUT |= PWM_PIN;

	    beatHeart(100);
    //    P7OUT &= ~PWM_PIN;

	  //  analogMode();
	   // setAndPlay(118);
	    setAndPlay(98);
        beatHeart(300);

        setAndPlay(1);
        beatHeart(300);

        setAndPlay(46);
        beatHeart(300);


        setAndPlay(46);
        beatHeart(300);

        setAndPlay(27);
        beatHeart(300);

        setAndPlay(93);
        beatHeart(300);
        setAndPlay(12);
        beatHeart(300);
        setAndPlay(16);
        beatHeart(300);
        setAndPlay(80);
        beatHeart(300);

	    beatHeart(500);

	}
}

