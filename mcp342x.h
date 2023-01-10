/*
 * Copyright 2021-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include "hal_config.h"
#include "hal_i2c.h"
#include "endpoints.h"
#include "rules.h"

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################

#define	mcp3422NUM_CHAN				2
#define	mcp3423NUM_CHAN				2
#define	mcp3424NUM_CHAN				4

// ######################################## Enumerations ###########################################

enum {													// I2C addresses options
	mcp342xAD_0	= 0x68,
	mcp342xAD_1,
	mcp342xAD_2,
	mcp342xAD_3,
	mcp342xAD_4,
	mcp342xAD_5,
	mcp342xAD_6,
	mcp342xAD_7,
} ;

enum { mcp342xR12_240, mcp342xR14_60, mcp342xR16_15, mcp342xR18_3_75 } ;// Resolution options

enum { mcp342xG1, mcp342xG2, mcp342xG4, mcp342xG8 } ;	// Gain options

enum { mcp342xR0, mcp342xR1, mcp342xR2, mcp342xCFG } ;	// Read Data & Config Index

//		Disabled	Volts		mAmps	Ohms
enum { mcp342xM0, mcp342xM1, mcp342xM2, mcp342xM3 } ;

// ######################################### Structures ############################################

typedef union mcp342x_cfg_t {
	struct __attribute__((packed)) {
/*LSB*/	u8_t		PGA		: 2 ;						// Gain 1, 2, 4 or 8
		u8_t		RATE	: 2 ;						// Rate 240/12, 60/14, 15/16 or 3.75/18
		u8_t		OS_C	: 1 ;						// Mode 0=OneShot, 1=Continuous
		u8_t		CHAN	: 2 ;						// Channel 0 -> 3
/*MSB*/	u8_t		nRDY	: 1 ;						// RD=ReaDY, WR=1 for Convert
	} ;
	u8_t		Conf ;
} mcp342x_cfg_t ;
DUMB_STATIC_ASSERT(sizeof(mcp342x_cfg_t) == 1) ;

typedef struct {
	i2c_di_t * psI2C;
	SemaphoreHandle_t mux;
	TimerHandle_t th;
	StaticTimer_t ts;
	struct __attribute__((packed)) {
		u8_t I2Cnum:4;				// index into I2C Device Info table
		u8_t ChLo:4;
		u8_t ChHi:4;
		u8_t NumCh:3;				// 1, 2 or 4
		u32_t Spare:17;
	} ;
	mcp342x_cfg_t Chan[4];
	u32_t Modes;								// 16 x 2-bit flags, 2 per channel
} mcp342x_t ;
DUMB_STATIC_ASSERT(sizeof(mcp342x_t) == (sizeof(i2c_di_t *) + sizeof(SemaphoreHandle_t) + 60));

// ####################################### Public functions ########################################

int	mcp342xReadHdlr(epw_t *) ;
int mcp342xConfigMode(rule_t * psR, int Xcur, int Xmax);

int	mcp342xIdentify(i2c_di_t *) ;
int	mcp342xConfig(i2c_di_t *) ;
void	mcp342xReConfig(i2c_di_t * psI2C_DI) ;

int	mcp342xReportChan(u8_t) ;
int	mcp342xReportDev(mcp342x_t *) ;
int	mcp342xReportAll(void) ;

#ifdef __cplusplus
}
#endif
