#include "msp.h"


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
 *          SDA                  P6.6     Tertiary
 *          SCL                  P6.7     Tertiary
 *
 *    LRA (disk motor) focused.
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
}
