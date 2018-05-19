/* Registers */

#define STATUS_R					(0x00)
#define OC_DETECT					(0x00)		
#define OVER_TEMP					(0x02)		
#define DIAG_RESULT					(0x08)		
#define DEVICE_ID					(0x20)

#define MODE_R 						(0x01)
#define MODE						(0x00)
#define STANDBY						(0x40)	//2^6 ?? double check
#define DEV_RESET					(0x80) 	//2^7


#define RT_PLAYBACK_Q_R				(0x02)

#define LIBRARY_R					(0x03)

#define WAVE_A_SEQ_R				(0x04)
#define WAVE_B_SEQ_R				(0x05)
#define WAVE_C_SEQ_R				(0x06)
#define WAVE_D_SEQ_R				(0x07)
#define WAVE_E_SEQ_R				(0x08)
#define WAVE_F_SEQ_R				(0x09)
#define WAVE_G_SEQ_R				(0x0A)
#define WAVE_H_SEQ_R				(0x0B)


#define GO_R						(0x0C)

#define OVERDRIVE_TIME_OFFSET_R 	(0x0D)

#define SUSTAIN_TIME_OFFSET_POS_R	(0x0E)

#define SUSTAIN_TIME_OFFSET_NEG_R 	(0x0F)

#define BRAKE_TIME_OFFSET_R			(0x10)

#define AUDIO_TO_VIBE_R				(0x11)
#define AUDIO_MIN_IN_R				(0x12)
#define AUDIO_MAX_IN_R				(0x13)
#define ADUIO_MIN_OUT_R				(0x14)
#define AUDIO_MAX_OUT_R				(0x15)


#define RATED_VOLTAGE_R				(0x16)
#define RATED_VOLTAGE				(0x00)

#define OVERDRIVE_CLAMP_V_R			(0x17)
#define OD_CLAMP					(0x00)

#define AUTO_CAL_COMP_RESULT_R		(0x18)
#define AUTO_CAL_COMP_RESULT		(0x00)

#define AUTO_CAL_BEMF_RESULT_R		(0x19)
#define AUTO_CAL_BEMF_RESULT		(0x00)

#define FB_CTRL_R					(0x1A)
#define BEMF_GAIN					(0x00)
#define LOOP_GAIN					(0x04)
#define FB_BRAKE_FACTOR				(0x10)
#define N_ERM_LRA 					(0x80)

#define CTRL_1_R					(0x1B)
#define DRIVE_TIME					(0x00)
#define AC_COUPLE					(0x20)
#define STARTUP_BOOST				(0x80)


#define CTRL_2_R					(0x1C)
#define IDISS_TIME_RMA				(0x00)
#define BLANKING_TIME_RMA			(0x04)
#define SAMPE_TIME_RMA				(0x20)
#define BRAKE_STABILIZER_RMA		(0x40)
#define BIDIR_INPUT_RMA	 			(0x80)


#define CTRL_3_R					(0x1D)
#define LRA_OPEN_LOOP				(0x00)
#define N_PWM_ANALOG	 			(0x02)
#define LRA_DRIVE_MODE	 			(0x04)
#define DATA_FORMATE_RTP			(0x08)
#define SUPPLY_COMP_DIS 			(0x10)
#define ERM_OPEN_LOOP		 		(0x20)
#define NG_THRESH		 			(0x40)

#define CTRL_4_R					(0x1E)
#define AUTO_CAL_TIME		 		(0x10)
#define ZC_DET_TIME		 			(0x40)

#define CTRL_5_R					(0x1F)
#define IDISSTIME_LRA				(0x00)
#define BLANKING_TIME_LRA			(0x04)
#define PLAYBACK_INTERVAL_LRA		(0x10)
#define LRA_AUTO_OPEN_LOOP			(0x20)
#define AUTO_OL_CNT					(0x80)

#define OL_LOOP_PERIOD_R			(0x20)
#define OL_LRA_PERIOD				(0x00)

#define VOLTAGE_MONITOR_R			(0x21)
#define VBAT						(0x00)

#define LRA_RESONANCE_R				(0x22)
#define LRA_PERIOD					(0x00)

/* PRE Calibration Setup */
 


