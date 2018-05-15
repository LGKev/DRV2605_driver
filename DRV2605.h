/*
 * DRV2605.h
 *
 *  Created on: May 11, 2018
 *      Author: Kevin Kuwata
 *      gitHub: github.com/LGKev
 *
 */

#include "msp.h"

#ifndef DRV2605_H_
#define DRV2605_H_

#define DRV_DEFAULT_ADDRESS         (0x5A)        //7bit addr

/*
 * @name: initDriver(void);
 * @brief: configures gpio pins, i2c bus UCB3 , and the TA.2 for pwm.
 * @inputs: none
 * @return: none
 * */
void initDriver(void);


/*
 * @name: uint_8t DRVSingleWrite(uint8_t registerToWrite, uint8_t valueToWrite);
 * @brief: writes a single value to a specified register at the GLOBALLY DEFINED address.
 * @inputs: byte, registerToWrite and byte valueToWrite
 * @global: DRV2605 address
 *
 * @returns 0 if failed: ie no ack
 * */
uint8_t DRVSingleWrite(uint8_t registerToWrite, uint8_t valueToWrite);



void beginTransmission(uint8_t address);

void stopTransmission(void);

void writeRegister(uint8_t address, uint8_t reg, uint8_t value);

void readRegister(uint8_t address, uint8_t reg, uint8_t numBytes);


#endif /* DRV2605_H_ */

/* Note:
 * Section 7.3.5.4 the Ref Voltage is 1.8 so 1.8 analog voltage is 100% and 0.9 is 50% to the motor, must be in this mode, otherwise in trig mode
 *
 * Libraries available are only LRA 6 and 7
 *
 * Setting GO BIT in Register 0x0C starts the wave form. and stops it by clearing the bit.
 *
 *  Basic Operation: first tests, can we talk to it?
 *  1) EN pin must be high
 *      a) to talk to I2C EN must be HIGH
 *  2) full reset (power cycle) or write the bit DEV_RESET in the re
 *  gister 0x01
 *  3) changing modes:
 *      Table 2
 *  4) GO BIT starts a sequence (software trigger)
 *
 *  //set up looking at section 7.4.5
 *  closed loop requires calibration...  but everything is auto adjusted...
 *      Auto calibrate procedure.
 *      looks like 11 inputs/ bits need to be set
 *      then we get 4 outputs in registers with the calibration values for the specific motor.
 *
 *      TI is pretty damn good. I will like to use their parts in the future. Look at page 26 of the data sheet. full step by step instructions
 *      for auto calibration.
 *
 *  Section 7.5.7.2.5: Waveform Sequencer
 *      1) load effect/sequence into waveform sequencer
 *      2) trigger with the GO bit.
 *
 *
 *
 *  TODO:
 *  1) need to write driver functions for:
 *      a) single write read
 *      b  single write write
 *      c) multiple write read
 *      d) multiple byte write
 *
 *
 *
 *
 *  Standby Bit register 0x01, Power on Reset (POR) value is asserted (set 1), allows for i2c com, and retains register values!
 *          clear this bit to go to ready mode.
 *
 *  Check out state Machine 7.4.1: if EN is 0 the device goes to shutdown mode, all registers are going to lose their values.
 *          save and restore before and after (respectively).
 *
 *  If nothing happens, check the OC_DETECT bit to see if there is a short.
 *
 *  //ack at 0x5A, but in LA should be 0xB4 or 0xB5 for a read or a write.
 *
 * */
