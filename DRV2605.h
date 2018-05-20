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
/* ======================================================================================================================================================*/
                                                /* Register Defines*/
/* ======================================================================================================================================================*/

#define STATUS_R                    (0x00)
#define OC_DETECT                   (0x00)
#define OVER_TEMP                   (0x02)
#define DIAG_RESULT                 (0x08)
#define DEVICE_ID                   (0x20)

#define MODE_R                      (0x01)
#define AUTO_CALIBRATION_MODE       (0X07)
#define MODE                        (0x00)
#define STANDBY                     (0x40)  //2^6 ?? double check
#define DEV_RESET                   (0x80)  //2^7
#define MODE_ANALOG_INPUT           (0x03)


#define RT_PLAYBACK_Q_R             (0x02)

#define LIBRARY_R                   (0x03)

#define WAVE_A_SEQ_R                (0x04)
#define WAVE_B_SEQ_R                (0x05)
#define WAVE_C_SEQ_R                (0x06)
#define WAVE_D_SEQ_R                (0x07)
#define WAVE_E_SEQ_R                (0x08)
#define WAVE_F_SEQ_R                (0x09)
#define WAVE_G_SEQ_R                (0x0A)
#define WAVE_H_SEQ_R                (0x0B)


#define GO_R                        (0x0C)

#define OVERDRIVE_TIME_OFFSET_R     (0x0D)

#define SUSTAIN_TIME_OFFSET_POS_R   (0x0E)

#define SUSTAIN_TIME_OFFSET_NEG_R   (0x0F)

#define BRAKE_TIME_OFFSET_R         (0x10)

#define AUDIO_TO_VIBE_R             (0x11)
#define AUDIO_MIN_IN_R              (0x12)
#define AUDIO_MAX_IN_R              (0x13)
#define ADUIO_MIN_OUT_R             (0x14)
#define AUDIO_MAX_OUT_R             (0x15)


#define RATED_VOLTAGE_R             (0x16)
#define RATED_VOLTAGE               (0x00)

#define OVERDRIVE_CLAMP_V_R         (0x17)
#define OD_CLAMP                    (0x00)

#define AUTO_CAL_COMP_RESULT_R      (0x18)
#define AUTO_CAL_COMP_RESULT        (0x00)

#define AUTO_CAL_BEMF_RESULT_R      (0x19)
#define AUTO_CAL_BEMF_RESULT        (0x00)

#define FB_CTRL_R                   (0x1A)
#define BEMF_GAIN                   (0x00)
#define LOOP_GAIN                   (0x04)
#define FB_BRAKE_FACTOR             (0x10)
#define N_ERM_LRA                   (0x80)

#define CTRL_1_R                    (0x1B)
#define DRIVE_TIME                  (0x00)
#define AC_COUPLE                   (0x20)
#define STARTUP_BOOST               (0x80)


#define CTRL_2_R                    (0x1C)
#define IDISS_TIME_RMA              (0x00)
#define BLANKING_TIME_RMA           (0x04)
#define SAMPE_TIME_RMA              (0x20)
#define BRAKE_STABILIZER_RMA        (0x40)
#define BIDIR_INPUT_RMA             (0x80)


#define CTRL_3_R                    (0x1D)
#define LRA_OPEN_LOOP               (0x00)
#define N_PWM_ANALOG                (0x02)
#define LRA_DRIVE_MODE              (0x04)
#define DATA_FORMATE_RTP            (0x08)
#define SUPPLY_COMP_DIS             (0x10)
#define ERM_OPEN_LOOP               (0x20)
#define NG_THRESH                   (0x40)

#define CTRL_4_R                    (0x1E)
#define AUTO_CAL_TIME               (0x10)
#define ZC_DET_TIME                 (0x40)

#define CTRL_5_R                    (0x1F)
#define IDISSTIME_LRA               (0x00)
#define BLANKING_TIME_LRA           (0x04)
#define PLAYBACK_INTERVAL_LRA       (0x10)
#define LRA_AUTO_OPEN_LOOP          (0x20)
#define AUTO_OL_CNT                 (0x80)

#define OL_LOOP_PERIOD_R            (0x20)
#define OL_LRA_PERIOD               (0x00)

#define VOLTAGE_MONITOR_R           (0x21)
#define VBAT                        (0x00)

#define LRA_RESONANCE_R             (0x22)



#define PWM_PIN                     0b00100000 //bit5 for port 7 pin 5


/* ======================================================================================================================================================*/

/*
 * @name: initDriver(void);
 * @brief: configures gpio pins, i2c bus UCB3 , and the TA.2 for pwm.
 * @inputs: none
 * @return: none
 * */
void initDriver(void);

/* @name: preAutoCalibrationLRA(void)
 * @brief: sets up DRV for auto calibration of an LRA motor
 * @inputs: for now none, but it could take in the 11 parameters needed...
 *  which case passing a struct might be easier, like a calibration_t struct or something
 * @output: none, but in the future could return the calibration fail or success bit in the status register
 *          reads the status register bit 3?
 * */
void preAutoCalibrationLRA(void);


/*
 * @name: void autoCalibrationLRA(void)
 * @brief: run this before playing waveforms, auto calibrates the specific motor to the driver
 * @input: none
 * @output: none
 * starts the calibration
 */
void autoCalibrationLRA(void);

/*  @name: void autoCalibrationERM(void)
 *  @brief: auto calibration for an ERM motor, 3.3V peak and 3v rated erm
 *  @inputt: none, values were calculated from this pdf:
 *  http://www.ti.com/lit/an/sloa189/sloa189.pdf
 *      for OD = A4 (3.6V) and Voltage Rating =90(3v)
 *
 */
void autoCalibrationERM(void);

/*
 * @name: analogMode()
 * @brief: sets the driver to receive analog voltage, does so until no longer in active mode. user must call standbyMode()
 * to stop playback
 * 1.8V is 100%
 * 0.9v is 50%
 * 0v is 0%
 * */
void analogMode(void);


/*
 * @name: uint_8t DRVSingleWrite(uint8_t registerToWrite, uint8_t valueToWrite);
 * @brief: writes a single value to a specified register at the GLOBALLY DEFINED address.
 * @inputs: byte, registerToWrite and byte valueToWrite
 * @global: DRV2605 address
 *
 * @returns 0 if failed: ie no ack
 * */
uint8_t DRVSingleWrite(uint8_t registerToWrite, uint8_t valueToWrite);

/* @name: setGoBit(void)
 * @brief: sets the go big to a 1, triggers playback
 * @input: none
 * @output: none
 * */
void setGoBit(void);

/* @name: setAndPlay(uint9_t waveformID)
 * @brief: sets the waveform id from table into the memory for playback, sets the go bit, and plays
 * @input: valid waveform id from 1  to 123
 * @output: vibration
 * Sets only one memory location. comes in and out of standby
 * */
void setAndPlay(uint8_t waveformID);


typedef struct{
    uint8_t bemf_gain;
    uint8_t A_cal_comp;
    uint8_t a_cal_bemf;
    uint8_t diag_result;
}autoCalibrationResults;

/* @name: getCalibrationValues(struct autoCalibrationResults result)
 * @brief: reads the registers 0x
 *  - bemf_gain (register 0x1A) bits 1 and 0
 *  - A_cal_comp (register  0x18)
 *  - a_cal_bemf (0x19)
 *  - diag_result == successful  calibration?
 * @input: pointer to the struct
 * @output: stores the results of calibration into the struct,
 *          returns a pointer
*/
/* struct getCalibrationValues(struct calibrationResults){

} */



void beginTransmission(uint8_t address);

void stopTransmission(void);

void writeRegister(uint8_t address, uint8_t reg, uint8_t value);

//void readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* array);

uint8_t readRegister(uint8_t address, uint8_t reg);


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
