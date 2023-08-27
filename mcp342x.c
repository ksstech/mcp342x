/*
 * Copyright (c) 2021-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"

#if (halHAS_MCP342X)
#include "mcp342x.h"
#include "FreeRTOS_Support.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "x_errors_events.h"
#include "x_string_general.h"

#define	debugFLAG					0xF000

#define	debugCONVERT				(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################

/* Test at 400KHx I2C speed, maybe add auto detect and step up mode in SCAN routine?
 *
 * MCP342X logic:
 * Have 4 channels (0-3) to deal with
 * Chip can either work with all channels same config or different config
 * Gain, resolution/sample rate can be different for each channel.
 * Input can be single ended or differential.
 * Since channel values has to be written for each read, and sample/convert can only then start
 * might as well support completely different modes (V/A/R) for each channel...
 *
 * If we allow completely different mode configurations for each channel on the same device, each
 * channel must be enumerated as a separate endpoint wrt voltage/current/resistance/power/energy
 *
 * Basic functionality:
 * Each channel treated as a separate (mode/sense/log) endpoint
 * Process:
 *	Lock all EWS related to specific device, effectively the specific device
 * 	Write device with full config (channel+gain+rate+mode)
 *	Wait for period based on rate/resolution specified
 *	Read the sampled value and unlock all EWS on specifcic device
 *	Convert the sample and store the calculated value in the endpoint structure
 *
 *	mode /mcp342x idx mode resolution gain {offset factor}
 *				   |	|		|		|	  |		|
 *				   |	|		|		|	  |		*- float multiplier
 *				   |	|		|		|	  *------- float adjustment
 *				   |	|		|		*------------- 1,	2,	4,	8
 *				   |	|		*--------------------- 12,	14,	16,	18
 *				   |	*----------------------------- V, 	A,	R,	X
 *				   *---------------------------------- 0,	1,	2,	3, 255
 */

// ###################################### General macros ###########################################

#define	mcp342X_BUSY				1					// release late = 0, early = 1

#define	MCP342X_T_SNS_MIN			250
#define	MCP342X_T_SNS				15000

// ###################################### Local variables ##########################################

mcp342x_t *	psaMCP342X = NULL;
epw_t *	psaMCP342X_EP = NULL;
u8_t	mcp342xNumDev = 0, mcp342xNumCh	= 0;

// ################################ Forward function declaration ###################################

epw_t * mcp342xGetWork(int x);
void	mcp342xSetDefault(epw_t * psEWP, epw_t *psEWS);
void	mcp342xSetSense(epw_t * psEWP, epw_t * psEWS);

// ######################################### Constants #############################################

const u16_t mcp342xDelay[4] = {
	5,								// 12 bit	1000 / 240	4.167mS
	17,								// 14 bit	1000 / 60	16.667mS
	67,								// 16 bit	1000 / 15	66.667mS
	267,							// 18 bit	1000 / 3.75	266.667mS
};

const vt_enum_t	sMCP342XFunc = {
	.work	= mcp342xGetWork,
	.reset	= mcp342xSetDefault,
	.sense	= mcp342xSetSense,
	.report = NULL
};

// ###################################### Private functions ########################################

/* When comparing read logic of DS18X20 with MCP342X, a key difference lies in the
 * fact that the DS18X20 devices can all be triggered to sample and convert in parallel.
 * In contrast each A2D channel on the MCP342X must be triggered separately followed by
 * a delay period proportional to the resolution used.
 * With the current blocking I2C driver it is not possible to interleave the sequence
 * with other I2C device activities, hence (for now) all 4 channels are triggered &
 * sampled sequentially at the rate of the fastest 1.
 */

int	mcp342xMap2Dev(int LogCh) {
	for (int dev = 0; dev < mcp342xNumDev; ++dev) {
		if (INRANGE(psaMCP342X[dev].ChLo, LogCh, psaMCP342X[dev].ChHi)) return dev;
	}
	IF_myASSERT(debugRESULT, 0);
	return erFAILURE;
}

void mcp342xSetBusy(int Dev, bool fBusy) {
	for (int i = 0; i < mcp342xNumCh; ++i) {			// mark ALL EWS on current device busy
		if (INRANGE(psaMCP342X[Dev].ChLo, i, psaMCP342X[Dev].ChHi)) psaMCP342X_EP[i].fBusy = fBusy;
	}
}

epw_t * mcp342xGetWork(int ch) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaMCP342X_EP) && (ch < mcp342xNumCh));
	return &psaMCP342X_EP[ch];
}

// Stop sensing on EWP level since vEpConfigReset() will handle EWx
void mcp342xSetDefault(epw_t * psEWP, epw_t * psEWS) {
//	psEWP->Rsns = 0;
}

/* When we get here the psEWS structure will already having been configured with the
 * parameters as supplied, just check & adjust for validity & new min Tsns */
void mcp342xSetSense(epw_t * psEWP, epw_t * psEWS) {
	// make sure newly specified EWS Tsns is not below the minimum
	if (psEWS->Tsns < MCP342X_T_SNS_MIN)
		psEWS->Tsns = MCP342X_T_SNS_MIN;
	// If the specific ESW Tsns is lower that current EWP Tsns, update EWP Tsns value
	if (psEWP->Tsns > psEWS->Tsns)
		psEWP->Tsns = psEWS->Tsns;
	// sense done on EWP level, discard EWS Tsns
	if (psEWP->fSECsns == 0)
		psEWS->Tsns = 0;
	// restart SNS timer
	psEWP->Rsns = psEWP->Tsns;
}

u8_t mcp342xBuf[4];

/**
 * @brief	step 3: sample read, convert  store
 * @param 	Expired timer handle
 */
void mcp342xReadCB(void * pvPara) {
	epw_t * psEWS = pvPara;
	u8_t ch	= psEWS->idx;							// Logical channel #
	mcp342xSetBusy(mcp342xMap2Dev(ch), 0);
	mcp342x_cfg_t sChCfg = { .Conf = mcp342xBuf[sizeof(mcp342xBuf)-1] };
	IF_EXEC_1(debugCONVERT, mcp342xReportChan, sChCfg.Conf);
	IF_PX(debugCONVERT, " [ %-'hhY ]", sizeof(mcp342xBuf), mcp342xBuf);
	if (sChCfg.RATE != mcp342xR18_3_75)
		mcp342xBuf[0] = (mcp342xBuf[1] & 0x80) ? 0xFF : 0x00;
	IF_PX(debugCONVERT, " [ %-'hhY ]", sizeof(mcp342xBuf), mcp342xBuf);
	int Raw = (mcp342xBuf[mcp342xR0] << 16) | (mcp342xBuf[mcp342xR1] << 8) | mcp342xBuf[mcp342xR2];
	x64_t X64;
	X64.x32[0].f32 = (float) Raw *  0.000015625;
	vCV_SetValueRaw(&psaMCP342X_EP[ch].var, X64);
	IF_P(debugCONVERT, " Raw=%d Norm=%f %s\r\n", Raw, X64.x32[0].f32, sChCfg.nRDY ? " (OLD sample)" : "");
}

/**
 * @brief	step 2: conversion timer expired, trigger sample read
 * @param 	(expired) timer handle
 */
void mcp342xTimerHdlr(TimerHandle_t xTimer) {
	epw_t * psEWS = pvTimerGetTimerID(xTimer);
	u8_t ch = psEWS->idx;							// Logical channel #
	u8_t dev = mcp342xMap2Dev(ch);					// Device #
	mcp342x_t * psMCP342X = &psaMCP342X[dev];
	int xLen = psMCP342X->Chan[ch - psMCP342X->ChLo].RATE == mcp342xR18_3_75 ? 4 : 3;
	halI2CM_Queue(psMCP342X->psI2C, i2cRC, NULL, 0, &mcp342xBuf[4-xLen], xLen, (i2cq_p1_t) mcp342xReadCB, (i2cq_p2_t) (void *) psEWS);
}

/**
 * @brief	step 1: trigger A->D conversion with delay
 * @param 	pointer to endpoint to be read
 */
int	mcp342xSense(epw_t * psEWx) {
	u8_t ch	= psEWx->idx;
	u8_t dev = mcp342xMap2Dev(ch);
	mcp342xSetBusy(dev, 1);
	// Address & configure channel, start conversion
	mcp342x_t * psMCP342X = &psaMCP342X[dev];
	mcp342x_cfg_t sChCfg = psMCP342X->Chan[ch - psMCP342X->ChLo];
	vTimerSetTimerID(psMCP342X->th, (void *) psEWx);	// make available to next stage
	return halI2CM_Queue(psMCP342X->psI2C, i2cWT, &sChCfg.Conf, sizeof(u8_t), NULL, 0,
			(i2cq_p1_t) psMCP342X->th, (i2cq_p2_t) (uint32_t) mcp342xDelay[sChCfg.RATE]);
}

int mcp342xConfigMode(rule_t * psR, int Xcur, int Xmax) {
	IF_RETURN_MX(psaMCP342X == NULL, "No MCP342X enumerated", erINV_OPERATION);
	u8_t AI = psR->ActIdx;
	u32_t mode = psR->para.x32[AI][0].u32;
	u32_t rate = psR->para.x32[AI][1].u32;
	u32_t gain = psR->para.x32[AI][2].u32;
	IF_P(debugTRACK && ioB1GET(dbgMode), "MCP342X Mode p0=%d p1=%lu p2=%lu p3=%lu\r\n", Xcur, mode, rate, gain);

	IF_RETURN_MX(mode > mcp342xM3 || rate > mcp342xR18_3_75 || gain > mcp342xG8, "Invalid mode/resolution/gain", erINV_PARA);
	do {
		int dev = mcp342xMap2Dev(Xcur);
		int ch = Xcur - psaMCP342X[dev].ChLo;
		psaMCP342X[dev].Chan[ch].PGA = gain;
		psaMCP342X[dev].Chan[ch].RATE = rate;
		maskSET2B(psaMCP342X[dev].Modes, ch, mode, u32_t);
	} while (++Xcur < Xmax);
	return erSUCCESS;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * mcp3424Identify() - device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	mcp342xIdentify(i2c_di_t * psI2C) {
	psI2C->TRXmS	= 20;
	psI2C->CLKuS = 400;			// Max 13000 (13mS)
	psI2C->Test	= 1;
	u8_t u8Buf[4];
	int iRV = halI2CM_Queue(psI2C, i2cR_B, NULL, 0, u8Buf, sizeof(u8Buf), (i2cq_p1_t) NULL, (i2cq_p2_t) (u32_t) 0);
	psI2C->Test = 0;
	IF_PX(debugTRACK && ioB1GET(ioI2Cinit), "mcp342x ID [ %-'hhY ]", sizeof(u8Buf), u8Buf);
	if ((iRV == erSUCCESS) && (u8Buf[3] == 0x90)) {
		psI2C->Type		= i2cDEV_MCP342X;
		// 5 bytes = 500uS @ 100KHz, 125uS @ 400Khz
		psI2C->Speed		= i2cSPEED_400;
		psI2C->DevIdx 	= mcp342xNumDev++;
		mcp342xNumCh		+= 4;						// MCP3424 specific
		IF_P(debugTRACK && ioB1GET(ioI2Cinit),"  Addr=0x%02X", psI2C->Addr);
	}
	return iRV;
}

int	mcp342xConfig(i2c_di_t * psI2C) {

int mcp342xReConfig(i2c_di_t * psI2C) {
	if (psaMCP342X == NULL) {							// 1st time here...
		IF_myASSERT(debugPARAM, psI2C->DevIdx == 0);
		// Primary endpoint init
		epw_t * psEWP = &table_work[URI_MCP342X];
		psEWP->var.def = SETDEF_CVAR(0, 1, vtVALUE, cvF32, 0, 1);
		psEWP->var.def.cv.vc	= mcp342xNumCh;
		psEWP->var.val.px.pv	= (void *) &sMCP342XFunc;
		psEWP->Tsns = psEWP->Rsns = MCP342X_T_SNS;
		psEWP->uri = URI_MCP342X;
		psEWP->fSECsns = 1;								// req due to delays, no parallel reads

		// Init secondary/enumerated endpoint
		psaMCP342X_EP = pvRtosMalloc(mcp342xNumCh * sizeof(epw_t));
		memset(psaMCP342X_EP, 0, mcp342xNumCh * sizeof(epw_t));
		for (int ch = 0; ch < mcp342xNumCh; ++ch) {
			epw_t * psEWS = &psaMCP342X_EP[ch];
			psEWS->var.def = SETDEF_CVAR(0, 1, vtVALUE, cvF32, 1, 0);
			psEWS->Tsns = psEWS->Rsns = MCP342X_T_SNS;
			psEWS->uri = URI_MCP342X;
			psEWS->idx = ch;
		}
		// Device array init
		psaMCP342X = pvRtosMalloc(mcp342xNumDev * sizeof(mcp342x_t));
		memset(psaMCP342X, 0, mcp342xNumDev * sizeof(mcp342x_t));
		mcp342xNumCh = 0;			// reset to start counting up again....
		IF_SYSTIMER_INIT(debugTIMING, stMCP342X, stMICROS, "MCP342X", 1, 300);
	}
	mcp342x_t * psMCP342X = &psaMCP342X[psI2C->DevIdx];
	psMCP342X->psI2C	= psI2C;
	psMCP342X->NumCh	= mcp3424NUM_CHAN;				// MCP3424 specific
	psMCP342X->ChLo		= mcp342xNumCh;				// MCP342X all models
	psMCP342X->ChHi		= psMCP342X->ChLo + psMCP342X->NumCh - 1;
	mcp342xNumCh		+= psMCP342X->NumCh;
	for (int ch = 0; ch < psMCP342X->NumCh; ++ch) {
		psMCP342X->Chan[ch].Conf = 0x90;
		psMCP342X->Chan[ch].CHAN = ch;
		maskSET2B(psMCP342X->Modes, ch, mcp342xM1, u32_t);	// default mode
	}
	// Default mode is 240SPS ie. 1000 / 240 = 4.167mS
	psMCP342X->th = xTimerCreateStatic("mcp342x", pdMS_TO_TICKS(5), pdFALSE, NULL, mcp342xTimerHdlr, &psMCP342X->ts);
	IF_P(debugTRACK && ioB1GET(ioI2Cinit)," %d of %d\r\n", psI2C_DI->DevIdx, mcp342xNumDev);
	return erSUCCESS;
}

int	mcp342xReportChan(report_t * psR, u8_t Value) {
	mcp342x_cfg_t sChCfg;
	sChCfg.Conf = Value;
	return wprintfx(psR, "  Cfg=0x%02X  nRDY=%d  C=%d  OS_C=%d  SAMP=%d  PGA=%d",
			sChCfg.Conf, sChCfg.nRDY, sChCfg.CHAN, sChCfg.OS_C, sChCfg.RATE, sChCfg.PGA);
}

int	mcp342xReportDev(report_t * psR, mcp342x_t * psMCP342X) {
	int iRV = 0;
	for (int ch = 0; ch < psMCP342X->NumCh; ++ch) {
		iRV += wprintfx(psR, "#%d - A=0x%02X", ch, psMCP342X->psI2C->Addr);
		iRV += mcp342xReportChan(psMCP342X->Chan[ch].Conf);
		int LogCh = psMCP342X->ChLo + ch;
		iRV += wprintfx(psR, "  L=%d  vNorm=%f\r\n", psMCP342X->ChLo + ch, xCV_GetValueScaled(&psaMCP342X_EP[LogCh].var, NULL).f64);
	}
	return iRV;
}

int	mcp342xReportAll(report_t * psR) {
	int iRV = 0;
	for (int dev = 0; dev < mcp342xNumDev; iRV += mcp342xReportDev(psR, &psaMCP342X[dev++]));
	iRV += xRtosReportTimer(&sR, m90e26TH);
	return iRV;
}
#endif
