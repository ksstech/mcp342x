/*
 * Copyright (c) 2021-24 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (HAL_MCP342X > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "mcp342x.h"
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
u8_t mcp342xNumDev = 0, mcp342xNumCh = 0;

// ################################ Forward function declaration ###################################

epw_t * mcp342xGetWork(int x);
void mcp342xSetDefault(epw_t * psEWP, epw_t *psEWS);
void mcp342xSetSense(epw_t * psEWP, epw_t * psEWS);

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
	.report = NULL,
};

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * mcp3424Identify() - device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	mcp342xIdentify(i2c_di_t * psI2C) {
	psI2C->Type = i2cDEV_MCP342X;
	psI2C->Speed = i2cSPEED_400;	// 5 bytes = 500uS @ 100KHz, 125uS @ 400Khz
	psI2C->TObus = 25;
	psI2C->Test	= 1;
	u8_t u8Buf[4];
	int iRV = halI2C_Queue(psI2C, i2cR_B, NULL, 0, u8Buf, sizeof(u8Buf), (i2cq_p1_t) NULL, (i2cq_p2_t) (u32_t) 0);
	if (iRV < erSUCCESS) goto exit;
	if (u8Buf[3] != 0x90) return erINV_WHOAMI;
	psI2C->DevIdx = mcp342xNumDev++;
	mcp342xNumCh += 4;						// MCP3424 specific
	psI2C->IDok = 1;
	psI2C->Test = 0;
	return iRV;
}

int	mcp342xConfig(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	if (psaMCP342X == NULL) {							// 1st time here...
		IF_myASSERT(debugPARAM, psI2C->DevIdx == 0);
		// Device array init
		psaMCP342X = pvRtosMalloc(mcp342xNumDev * sizeof(mcp342x_t));
		if (!psaMCP342X) return erNO_MEM;

		memset(psaMCP342X, 0, mcp342xNumDev * sizeof(mcp342x_t));
		mcp342xNumCh = 0;			// reset to start counting up again....
		IF_SYSTIMER_INIT(debugTIMING, stMCP342X, stMICROS, "MCP342X", 1, 300);
	}
	if (!psI2C->CFGok) {
		mcp342x_t * psMCP342X = &psaMCP342X[psI2C->DevIdx];
		psMCP342X->psI2C = psI2C;
		psMCP342X->NumCh = mcp3424NUM_CHAN;				// MCP3424 specific
		psMCP342X->ChLo = mcp342xNumCh;					// MCP342X all models
		psMCP342X->ChHi = psMCP342X->ChLo + psMCP342X->NumCh - 1;
		mcp342xNumCh += psMCP342X->NumCh;
		for (int ch = 0; ch < psMCP342X->NumCh; ++ch) {
			psMCP342X->Chan[ch].Conf = 0x90;
			psMCP342X->Chan[ch].CHAN = ch;
			maskSET2B(psMCP342X->Modes, ch, mcp342xM1, u32_t);	// default mode
		}
		// Default mode is 240SPS ie. 1000 / 240 = 4.167mS
		psMCP342X->th = xTimerCreateStatic("mcp342x", pdMS_TO_TICKS(5), pdFALSE, NULL, mcp342xTimerHdlr, &psMCP342X->ts);
	}
	psI2C->CFGok = 1;
exit:
	return iRV;
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
		iRV += mcp342xReportChan(psR, psMCP342X->Chan[ch].Conf);
		int LogCh = psMCP342X->ChLo + ch;
		iRV += wprintfx(psR, "  L=%d  vNorm=%f\r\n", psMCP342X->ChLo + ch, xCV_GetValueScaled(&psaMCP342X_EP[LogCh].var, NULL).f64);
	}
	return iRV;
}

int	mcp342xReportAll(report_t * psR) {
	int iRV = 0;
	for (int eCh = 0; eCh < mcp342xNumDev; ++eCh) {
		mcp342x_t * psMCP342X = &psaMCP342X[eCh];
		iRV += mcp342xReportDev(psR, psMCP342X);
		iRV += xRtosReportTimer(psR, psMCP342X->th);
	}
	return iRV;
}

#endif
