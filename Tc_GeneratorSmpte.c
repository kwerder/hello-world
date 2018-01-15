//****************************************************************************
//Programmer:	kwerder
//File name:	Tc_GeneratorSmpte.c
//Version:	1.0.0
//Date:		9/12/2013 - 1/15/2018
//Copyright (c) 2013-2018 Masterclock, Inc.
//Description:
//Mods:

#include "Tc_GeneratorSmpte.h"
#include "MC_types.h"
#include "MC_structs.h"
#include "debug_mc.h"
#include "stdioMcr.h"
#include "kinetis_io.h"
#include "mqx_interface.h"

//#include "ModuleSlave.h"
//#include "TcgModule_main.h"
#include "na_control_mcr.h"
#include "flextimer.h"
#include "SyncSystem.h"

//INT iSkewIndex=0;
//INT g_iSaveSkew[100];

extern UINT    g_uiFTM0_MsecReload;
extern RC600_CONTROL g_srRC600Control;
extern UINT64  g_ullIsrFTM3_OverflowOffset;
extern UINT64  g_ullIsrFtm3_TC_AtTopOfSecond;
extern UINT64  g_ullIsrFtm3_TC_MatchAtTopOfSecond;
extern UINT64  g_ullIsrFtm3_TC_MatchAtTopOfSecond_2; // kw rc1000

extern STimecodeTcgConfig g_srTcgConfig;
extern STimecodeTcgConfig g_srTcgConfig_2; // kw rc1000

STimecodeTcgConfig * gp_Tcg_Config; // kw rc1000

// kw v dropframe
extern UINT64 g_ullIsrFTM0_OverflowOffset;
// end kw

#define SMPTE_BITS_PER_FRAME                80

const UINT16 usDaysMonthElapsed[13] = {
	0,
	31, /* through January					*/
	59, /* through February (not including any leap year)	*/
	90, /* through March				 	*/
	120,/* through April					*/
	151,/* through May					    */
	181,/* through June					    */
	212,/* through July					    */
	243,/* through August					*/
	273,/* through September				*/
	304,/* through October					*/
	334,/* through November					*/
	365 /* through December					*/
};

const UINT16 usDaysMonthElapsedLeap[13] = {
	0,
	31, /* through January					*/
	60, /* through February (this and rest include leap day */
	91, /* through March					*/
	121,/* through April					*/
	152,/* through May					    */
	182,/* through June					    */
	213,/* through July					    */
	244,/* through August					*/
	274,/* through September				*/
	305,/* through October					*/
	335,/* through November					*/
	366 /* through December					*/
};


BOOLEAN g_bIsHalfBit;
BOOLEAN g_bDatatBit_Smpte;

// Skewing variables
SMPTE_SKEWING g_srSmpteGenSkewing;
UINT          g_uiSmpteMaxSkewPerBit;

SSmpteConfig g_srSmpteCurrentConfig;

#define MAX_SIZE_SMPTE_BUFFER 10
UINT8 g_aucSmpteNextSecondBuffer[MAX_SIZE_SMPTE_BUFFER];

UINT8 g_aucSmpteCurrentBuffer[MAX_SIZE_SMPTE_BUFFER];

const UINT8 g_aucSmpteCountOnesBCDLookUp[10] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2};

// see the SMPTE 309M-1999 Table 2 spec for the explanation of the codes
// code 01 - 12 : negative bias offset     Note: the 309M codes start at table index 1
const UINT8 gucSmpteOffsetTable1[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12};
// code 13 - 25 : positive bias offset     Note: the 309M codes start at table index 1
const UINT8 gucSmpteOffsetTable2[] = {0x00, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13};
// code 0A - 1F : negative bias with half hour offset
const UINT8 gucSmpteOffsetTable3[] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
// code 2A - 3F : positive bias with half hour offset
const UINT8 gucSmpteOffsetTable4[] = {0x3F, 0x3E, 0x3D, 0x3C, 0x3B, 0x3A, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B, 0x2A};

UINT16 g_usSmpteTicksPerBit;
UINT16 g_usSmpteTotalNumHalfBits;

UINT8 g_ucSmpteCurrentByte;
UINT8 g_ucSmpteCurrentBitMask;
UINT8 g_ucSmpteCurrentBitNumber;

UINT8 g_ucSmpteFrame_Counter;
UINT8 g_ucSmpteFrame_Reset;
UINT8 g_ucSmpteFrame_ones;
UINT8 g_ucSmpteFrame_tens;
UINT8 g_ucSmpteTcgMaxFrameValue;

BOOLEAN g_bSmpteNextOutputStateIsHigh;

UINT8 g_ucPolarity;

UINT16 g_usSmpteMatchValue;

UINT16 g_usSmpteModulatorSkewPreload;

// PROTOTYPES *****************************************************************
BOOLEAN fnSetSmpteStart(void);
void    fnSmpteGeneratorSet309M_TimeZone(INT i_Smpte_Channel); // kw rc1000 added parm
void    fnSmpteGeneratorSet309M_BinaryGroupFlags(void);
void    fnSmpteGeneratorSetMasterclockControlBits(void);

BOOLEAN fnSetSmpteStart_2(void); // kw rc1000
void    fnSmpteGeneratorSet309M_TimeZone_2(void); // kw rc1000
void    fnSmpteGeneratorSetMasterclockControlBits_2(void); // kw rc1000

void fnSmpteGeneratorTimerIsr_2(BOOLEAN bIsRolloverAdded_2); // kw rc1000

BOOLEAN fnLockSmpte(char *pName); // kw rc1000
BOOLEAN fnUnlockSmpte(char *pName); // kw rc1000

// kw v - dropframe
BOOLEAN g_bDropFrameGenerating; 
BOOLEAN g_bDropFrameTopOfMinute;
BOOLEAN g_bDropFrameFramesDropped;
BOOLEAN g_bDropFrameJam;
//BOOLEAN g_iNextTicks; // kw testing
//BOOLEAN g_iNextEdge; // kw testing

//UINT32 g_uiFrame_usec; // kw testing
//UINT32 g_uiFrame_count; // kw testing
//UINT32 g_uiLength, g_uiTime2, g_uiTime1; // kw testing
//UINT32 g_uiDF_err;
//UINT32 g_uiDF_err_per_frame;
//UINT g_uiDF_Skew_Modulus, g_uiDF_Skew_Modulus_2, g_uiDF_Skew_Modulus_3;
//UINT g_uiDF_Skew_Modulus_Saved;
//BOOLEAN g_bDF_add_skew;
//BOOLEAN g_bDF_more_skew;
//BOOLEAN g_bDF_less_skew;

UINT g_uiSmpteTosSum, g_uiSmpteTosVariance, g_uiSmpteTosCount; // holds sum of difference between smpte and system TOSs.
UINT g_uiSmpteTosGreatest, g_uiSmpteTosLeast, g_ui_mS;
INT g_iSmpteTosPrevious;

//BOOLEAN gb_DropFrameTimedStart, gb_DropFrameSync;
BOOLEAN g_bDF_Sync_Now;

UINT8 g_ucEdgeNumber;
UINT8 g_aucDFskew[DF_SKEW_TEMP_MAX];
UINT8 g_aucDFskew_temp[DF_SKEW_TEMP_MAX];


SSmpteConfig g_srSmpteCurrentConfig_2;  // kw rc1000
UINT8 g_aucSmpteNextSecondBuffer_2[MAX_SIZE_SMPTE_BUFFER]; // kw rc1000
UINT8 g_aucSmpteCurrentBuffer_2[MAX_SIZE_SMPTE_BUFFER];  // kw rc1000

extern UINT64  g_ullIsrFtm3_TC_MatchAtTopOfSecond_2; // kw rc1000

BOOLEAN         g_bDatatBit_Smpte_2; // kw rc1000
BOOLEAN         g_bIsHalfBit_2; // kw rc1000
BOOLEAN         g_bSmpteNextOutputStateIsHigh_2; // kw rc1000

UINT8           g_ucEdgeNumber_2; // kw rc1000
UINT8           g_aucDFskew_2[DF_SKEW_TEMP_MAX]; // kw rc1000
UINT8           g_aucDFskew_temp_2[DF_SKEW_TEMP_MAX]; // kw rc1000

SMPTE_SKEWING g_srSmpteGenSkewing_2; // kw rc1000
UINT          g_uiSmpteMaxSkewPerBit_2; // kw rc1000
UINT16 g_usSmpteModulatorSkewPreload_2; // kw rc1000

SMPTE_SKEWING   g_srSmpteGenSkewing_2; // kw rc1000
UINT16          g_usSmpteMatchValue_2; // kw rc1000

UINT8           g_ucSmpteFrame_Counter_2; // kw rc1000
UINT8           g_ucSmpteCurrentBitNumber_2; // kw rc1000
UINT8           g_ucSmpteTcgMaxFrameValue_2; // kw rc1000

UINT16 g_usSmpteTicksPerBit_2; // kw rc1000
UINT16 g_usSmpteTotalNumHalfBits_2; // kw rc1000

UINT8 g_ucSmpteCurrentByte_2; // kw rc1000
UINT8 g_ucSmpteCurrentBitMask_2; // kw rc1000
UINT8 g_ucSmpteCurrentBitNumber_2; // kw rc1000

UINT8 g_ucSmpteFrame_Counter_2; // kw rc1000
UINT8 g_ucSmpteFrame_Reset_2; // kw rc1000
UINT8 g_ucSmpteFrame_ones_2; // kw rc1000
UINT8 g_ucSmpteFrame_tens_2; // kw rc1000
UINT8 g_ucSmpteTcgMaxFrameValue_2; // kw rc1000

BOOLEAN g_bSmpteNextOutputStateIsHigh_2; // kw rc1000

UINT8 g_ucPolarity_2; // kw rc1000

UINT16 g_usSmpteMatchValue_2; // kw rc1000

UINT32 g_ulIsrFtm3_LastDiff_2[FTM3_MAX_LAST_DIFF]; // kw rc1000

BOOLEAN g_bDF_Sync_Now_2; // kw rc1000
BOOLEAN g_bDropFrameGenerating_2; // kw rc1000
BOOLEAN g_bDropFrameTopOfMinute_2; // kw rc1000
BOOLEAN g_bDropFrameFramesDropped_2; // kw rc1000
BOOLEAN g_bDropFrameJam_2; // kw rc1000

UINT8 * gp_SmpteNextSecondBuffer; // kw rc1000
SSmpteConfig * gp_SmpteCurrentConfig; // kw rc1000

//UINT64  g_ullDF_MeasuredTicksPerSecond, g_ullDFticks2, g_ullDFticks1; // kw v dropframe
// end kw


//*****************************************************************************
//      Name:           fnSmpteSetConfig()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSetConfig(SSmpteConfig *psrConfig)
{

    UINT32 ulSmpteTimerInputFrequency;

    SDateTimeUsec srInitDateTime;

    if (g_uiFtmFreq == FTM_IS_15Mhz)
    {
        ulSmpteTimerInputFrequency = 15000000; // 15 Mhz
    } else
    {
        ulSmpteTimerInputFrequency = 10000000; // 10 Mhz
    }

    memset(g_aucSmpteNextSecondBuffer, 0, sizeof(g_aucSmpteNextSecondBuffer));
    memset(g_aucSmpteCurrentBuffer,    0, sizeof(g_aucSmpteCurrentBuffer));

    // make sure all SMPTE variabes are set to a known value;
    g_bIsHalfBit 				 = true; // the next half bit is not the start of the bit
    g_bDatatBit_Smpte            = false;

    memset(&g_srSmpteCurrentConfig,0,sizeof(g_srSmpteCurrentConfig));
    memset(g_aucSmpteNextSecondBuffer,0,sizeof(g_aucSmpteNextSecondBuffer));
    memset(g_aucSmpteCurrentBuffer,0,sizeof(g_aucSmpteCurrentBuffer));

    g_usSmpteTicksPerBit = 0;
    g_usSmpteTotalNumHalfBits = 0;

    g_ucSmpteFrame_ones = 0;
    g_ucSmpteFrame_tens = 0;

    g_ucPolarity = 0;

    g_usSmpteMatchValue = 0;

    g_usSmpteModulatorSkewPreload = 0;

    // end of init to zero

    g_ucSmpteFrame_Counter       = 0;
    g_ucSmpteTcgMaxFrameValue    = SMPTE_FRAME_RATE_30 - 1;
    g_ucSmpteCurrentBitMask      = 0x01;
    g_ucSmpteCurrentByte         = 0;
    g_ucSmpteCurrentBitNumber    = 1;

    // This should not be needed, set the date/time just to make sure we never send junk
    srInitDateTime.m_ucValid                 = true;
    srInitDateTime.m_srDate.m_sYear          = 2000;
    srInitDateTime.m_srDate.m_ucDate         = 1;
    srInitDateTime.m_srDate.m_ucMonth        = 2;
    srInitDateTime.m_srDate.m_ucDayOfWeek    = 4;
    srInitDateTime.m_srTime.m_ucHour         = 6;
    srInitDateTime.m_srTime.m_ucMinute       = 0;
    srInitDateTime.m_srTime.m_ucSecond       = 0;
    srInitDateTime.m_srTime.m_ulMicroSeconds = 0;

    // need semaphore
    // The code in SetNextDateTime() and the funcs it calls 
    // use these pointers to distinguish ch 1 and 2.
 // move into func
 //   gp_SmpteNextSecondBuffer = g_aucSmpteNextSecondBuffer; // kw rc1000
 //   gp_SmpteCurrentConfig = &g_srSmpteCurrentConfig; // kw rc1000
 //   gp_Tcg_Config = &g_srTcgConfig; // kw rc1000
    
    // This is called to get the default date-time into the frame as a starting point.
    // In normal operation fnTcgSysSetNextDateTime() calls this func each second with the actual date-time.
    
    //fnSmpteGeneratorSetNextDateTime(&srInitDateTime);
    fnSmpteGeneratorSetNextDateTime(&srInitDateTime, SMPTE_CHANNEL_1); // kw rc1000
    
    g_bDropFrameGenerating = false; // kw v dropframe

    g_srSmpteCurrentConfig = *psrConfig;
    //g_ucSmpteMaxiumBits = NUMBER_OF_BITS_PER_SMPTE_FRAME;

	switch(g_srSmpteCurrentConfig.m_ucSmpteFrameRate)
	{
		case SMPTE_FRAME_RATE_24:
            g_usSmpteMatchValue            = ulSmpteTimerInputFrequency/SMPTE_24FPS_NUM_HALF_BITS;
            g_ucSmpteTcgMaxFrameValue      = SMPTE_FRAME_RATE_24 - 1;
            g_usSmpteTicksPerBit           = ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_24*SMPTE_BITS_PER_FRAME);

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload  = ulSmpteTimerInputFrequency - (g_usSmpteTicksPerBit*SMPTE_FRAME_RATE_24*SMPTE_BITS_PER_FRAME);

            g_usSmpteTotalNumHalfBits      = SMPTE_24FPS_NUM_HALF_BITS;

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_24))+(UINT)1;
			break;

		case SMPTE_FRAME_RATE_25:
            g_usSmpteMatchValue            = ulSmpteTimerInputFrequency/SMPTE_25FPS_NUM_HALF_BITS;
            g_ucSmpteTcgMaxFrameValue      = SMPTE_FRAME_RATE_25 - 1;
            g_usSmpteTicksPerBit           = ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_25*SMPTE_BITS_PER_FRAME);

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload  = ulSmpteTimerInputFrequency - (g_usSmpteTicksPerBit*SMPTE_FRAME_RATE_25*SMPTE_BITS_PER_FRAME);
            g_usSmpteTotalNumHalfBits      = SMPTE_25FPS_NUM_HALF_BITS;

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_25))+(UINT)1;
			break;

        case SMPTE_FRAME_RATE_30:
            
            g_usSmpteMatchValue            = ulSmpteTimerInputFrequency/SMPTE_30FPS_NUM_HALF_BITS; // 15000000 div by 4800 == 3125
            g_usSmpteTotalNumHalfBits      = SMPTE_30FPS_NUM_HALF_BITS;
			g_ucSmpteTcgMaxFrameValue      = SMPTE_FRAME_RATE_30 - 1;
            g_usSmpteTicksPerBit           = ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME); // 15m/30*80 = 6250

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload  = ulSmpteTimerInputFrequency - (g_usSmpteTicksPerBit*SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME);

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_30))+(UINT)1;
			break;
            
        case SMPTE_FRAME_RATE_DROPFRAME:
          
            g_bDropFrameGenerating = true;
              
            // Match value is put into uiSmpteMatchValue in gen timer ISR
            // where it is modified by skewing values then added to FTM3_C1V.
            // ulSmpteTimerInputFrequency is 10M or 15M
           
            g_usSmpteMatchValue            = ulSmpteTimerInputFrequency/SMPTE_DROPFRAME_NUM_HALF_BITS; // 15000000 div by 4795 == 3128.2586...
            // 15000000 div by 4794 == 3128.9111389 // 15000000 div by 4794 == 3128.258603
            g_usSmpteTotalNumHalfBits      = SMPTE_DROPFRAME_NUM_HALF_BITS;

			g_ucSmpteTcgMaxFrameValue      = SMPTE_FRAME_RATE_30 - 1;
			
			// Maybe mod this for df ???
            g_usSmpteTicksPerBit           = ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME); // 15m/30*80 = 6250

            // We have to add this many counts to keep SMPTE in sync with top of second
            //g_usSmpteModulatorSkewPreload  = ulSmpteTimerInputFrequency - (g_usSmpteTicksPerBit*SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME);
            // The 2nd part of the right is (ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME)) * SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME
            // which is just ulSmpteTimerInputFrequency, so the preload is always freq - freq = 0.

            // kw q: why 10 ms ??
            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_30))+(UINT)1;
			break;

	}

	// This will cause fnSetSmpteStart() start a new frame at the next top of second
	g_ucSmpteCurrentBitNumber = MAX_SMPTE_BITS + 2;
	g_ucSmpteFrame_Counter    = g_ucSmpteTcgMaxFrameValue + 2;
    
} // end Smpte Generator Set Config




// rc1000
//*****************************************************************************
//      Name:           fnSmpteSetConfig_2()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSetConfig_2(SSmpteConfig *psrConfig_2)
{

    UINT32 ulSmpteTimerInputFrequency_2;

    SDateTimeUsec srInitDateTime_2;

    if (g_uiFtmFreq == FTM_IS_15Mhz)
    {
        ulSmpteTimerInputFrequency_2 = 15000000; // 15 Mhz
    } else
    {
        ulSmpteTimerInputFrequency_2 = 10000000; // 10 Mhz
    }

    memset(g_aucSmpteNextSecondBuffer_2, 0, sizeof(g_aucSmpteNextSecondBuffer_2));
    memset(g_aucSmpteCurrentBuffer_2,    0, sizeof(g_aucSmpteCurrentBuffer_2));

    // make sure all SMPTE variabes are set to a known value;
    g_bIsHalfBit_2 				 = true; // the next half bit is not the start of the bit
    g_bDatatBit_Smpte_2            = false;

    memset(&g_srSmpteCurrentConfig_2,   0,sizeof(g_srSmpteCurrentConfig_2));
    memset(g_aucSmpteNextSecondBuffer_2,0,sizeof(g_aucSmpteNextSecondBuffer_2));
    memset(g_aucSmpteCurrentBuffer_2,   0,sizeof(g_aucSmpteCurrentBuffer_2));

    g_usSmpteTicksPerBit_2 = 0;
    g_usSmpteTotalNumHalfBits_2 = 0;

    g_ucSmpteFrame_ones_2 = 0;
    g_ucSmpteFrame_tens_2 = 0;

    g_ucPolarity_2 = 0;

    g_usSmpteMatchValue_2 = 0;

    g_usSmpteModulatorSkewPreload_2 = 0;

    // end of init to zero

    g_ucSmpteFrame_Counter_2       = 0;
    g_ucSmpteTcgMaxFrameValue_2    = SMPTE_FRAME_RATE_30 - 1;
    g_ucSmpteCurrentBitMask_2      = 0x01;
    g_ucSmpteCurrentByte_2         = 0;
    g_ucSmpteCurrentBitNumber_2    = 1;

    // This should not be needed, set the date/time just to make sure we never send junk
    srInitDateTime_2.m_ucValid                 = true;
    srInitDateTime_2.m_srDate.m_sYear          = 2000;
    srInitDateTime_2.m_srDate.m_ucDate         = 1;
    srInitDateTime_2.m_srDate.m_ucMonth        = 2;
    srInitDateTime_2.m_srDate.m_ucDayOfWeek    = 4;
    srInitDateTime_2.m_srTime.m_ucHour         = 6;
    srInitDateTime_2.m_srTime.m_ucMinute       = 0;
    srInitDateTime_2.m_srTime.m_ucSecond       = 0;
    srInitDateTime_2.m_srTime.m_ulMicroSeconds = 0;

    // need semaphore
    // The code in SetNextDateTime() and the funcs it calls 
    // use these pointers to distinguish ch 1 and 2.
 // move into func
 //   gp_SmpteNextSecondBuffer = g_aucSmpteNextSecondBuffer_2;
 //   gp_SmpteCurrentConfig = &g_srSmpteCurrentConfig_2;
 //   gp_Tcg_Config = &g_srTcgConfig_2;
    
    // This is called to get the default date-time into the frame as a starting point.
    // In normal operation fnTcgSysSetNextDateTime() calls this func each second with the actual date-time.
    
    //fnSmpteGeneratorSetNextDateTime(&srInitDateTime_2);
    fnSmpteGeneratorSetNextDateTime(&srInitDateTime_2, SMPTE_CHANNEL_2);
    
    g_bDropFrameGenerating_2 = false; // kw v dropframe

    g_srSmpteCurrentConfig_2 = *psrConfig_2;
    //g_ucSmpteMaxiumBits = NUMBER_OF_BITS_PER_SMPTE_FRAME;

	switch(g_srSmpteCurrentConfig_2.m_ucSmpteFrameRate)
	{
		case SMPTE_FRAME_RATE_24:
            g_usSmpteMatchValue_2            = ulSmpteTimerInputFrequency_2/SMPTE_24FPS_NUM_HALF_BITS;
            g_ucSmpteTcgMaxFrameValue_2      = SMPTE_FRAME_RATE_24 - 1;
            g_usSmpteTicksPerBit_2           = ulSmpteTimerInputFrequency_2/(SMPTE_FRAME_RATE_24*SMPTE_BITS_PER_FRAME);

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload_2  = ulSmpteTimerInputFrequency_2 - (g_usSmpteTicksPerBit_2*SMPTE_FRAME_RATE_24*SMPTE_BITS_PER_FRAME);

            g_usSmpteTotalNumHalfBits_2      = SMPTE_24FPS_NUM_HALF_BITS;

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit_2         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_24))+(UINT)1;
			break;

		case SMPTE_FRAME_RATE_25:
            g_usSmpteMatchValue_2            = ulSmpteTimerInputFrequency_2/SMPTE_25FPS_NUM_HALF_BITS;
            g_ucSmpteTcgMaxFrameValue_2      = SMPTE_FRAME_RATE_25 - 1;
            g_usSmpteTicksPerBit_2           = ulSmpteTimerInputFrequency_2/(SMPTE_FRAME_RATE_25*SMPTE_BITS_PER_FRAME);

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload_2  = ulSmpteTimerInputFrequency_2 - (g_usSmpteTicksPerBit_2*SMPTE_FRAME_RATE_25*SMPTE_BITS_PER_FRAME);
            g_usSmpteTotalNumHalfBits_2      = SMPTE_25FPS_NUM_HALF_BITS;

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit_2         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_25))+(UINT)1;
			break;

        case SMPTE_FRAME_RATE_30:
            
            g_usSmpteMatchValue_2            = ulSmpteTimerInputFrequency_2/SMPTE_30FPS_NUM_HALF_BITS; // 15000000 div by 4800 == 3125
            g_usSmpteTotalNumHalfBits_2      = SMPTE_30FPS_NUM_HALF_BITS;
			g_ucSmpteTcgMaxFrameValue_2      = SMPTE_FRAME_RATE_30 - 1;
            g_usSmpteTicksPerBit_2           = ulSmpteTimerInputFrequency_2/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME); // 15m/30*80 = 6250

            // We have to add this many counts to keep SMPTE in sync with top of second
            g_usSmpteModulatorSkewPreload_2  = ulSmpteTimerInputFrequency_2 - (g_usSmpteTicksPerBit_2*SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME);

            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit_2         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_30))+(UINT)1;
			break;
            
        case SMPTE_FRAME_RATE_DROPFRAME:
          
            g_bDropFrameGenerating = true;
              
            // Match value is put into uiSmpteMatchValue in gen timer ISR
            // where it is modified by skewing values then added to FTM3_C1V.
            // ulSmpteTimerInputFrequency is 10M or 15M
           
            g_usSmpteMatchValue_2            = ulSmpteTimerInputFrequency_2/SMPTE_DROPFRAME_NUM_HALF_BITS; // 15000000 div by 4795 == 3128.2586...
            // 15000000 div by 4794 == 3128.9111389 // 15000000 div by 4794 == 3128.258603
            g_usSmpteTotalNumHalfBits_2      = SMPTE_DROPFRAME_NUM_HALF_BITS;

			g_ucSmpteTcgMaxFrameValue_2      = SMPTE_FRAME_RATE_30 - 1;
			
			// Maybe mod this for df ???
            g_usSmpteTicksPerBit_2           = ulSmpteTimerInputFrequency_2/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME); // 15m/30*80 = 6250

            // We have to add this many counts to keep SMPTE in sync with top of second
            //g_usSmpteModulatorSkewPreload  = ulSmpteTimerInputFrequency - (g_usSmpteTicksPerBit*SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME);
            // The 2nd part of the right is (ulSmpteTimerInputFrequency/(SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME)) * SMPTE_FRAME_RATE_30*SMPTE_BITS_PER_FRAME
            // which is just ulSmpteTimerInputFrequency, so the preload is always freq - freq = 0.

            // kw q: why 10 ms ??
            // # ticks in 10 milleseconds / number of bits
            g_uiSmpteMaxSkewPerBit_2         = (((UINT)g_uiFTM0_MsecReload*(UINT)10)/((UINT)SMPTE_BITS_PER_FRAME*(UINT)SMPTE_FRAME_RATE_30))+(UINT)1;
			break;

	}

	// This will cause fnSetSmpteStart() start a new frame at the next top of second
	g_ucSmpteCurrentBitNumber_2 = MAX_SMPTE_BITS + 2;
	g_ucSmpteFrame_Counter_2    = g_ucSmpteTcgMaxFrameValue_2 + 2;
    
} // end Smpte Generator Set Config 2





//*****************************************************************************
//      Name:           fnSmpteGeneratorStart()
//
//      Description:    Start the SMPTE output synchronized to the top of second.
//                      NOTE::: this functions can only be called near the half-second
//                              to make sure we are not near the top of second.
//
//                      This is called from StartTcgOutput() at system start, 
//                      when tc config has changed, when jam syncing.
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorStart(void)
{
    #ifdef _DEBUG_TCG
    debugPrintf("Waiting to start SMPTE\n");
    #endif
    
    debugPrintf("fnSmpteGeneratorStart\n");

	// SMPTE bits always start with a high, get ready by setting the port low
    // If we are currently generating SMPTE, this will stop the time code output
    fnFtm3Ch1SetOutputSmpte(1);       // this will force the output high
    
    memset(g_ulIsrFtm3_LastDiff, 0, sizeof(g_ulIsrFtm3_LastDiff));

    // Make the frame. Get current time, apply TZ, DST, put into buffer.
    fnTcgSysSetNextDateTime(0); // smpte

    g_bDatatBit_Smpte              = false;  // the first bit is always a zero

    g_ucSmpteCurrentByte           = 0;
    g_ucSmpteCurrentBitMask        = 0x01;
    //g_ucSmpteCurrentBitNumber      = 1;      // the jam sync sends the first bit

    //g_ucSmpteFrame_Counter         = 0;

    g_bIsHalfBit   			   = true;  // the next half bit is not the start of the bit

    // This will cause fnSetSmpteStart() to start a new frame
    g_ucSmpteCurrentBitNumber = MAX_SMPTE_BITS + 2;
    g_ucSmpteFrame_Counter    = g_ucSmpteTcgMaxFrameValue + 2;

    // the first bit after a jam sync is always a zero, we do not toggle at the half bit.
    fnSetSmpteStart(); // make the frame - put in data

    // fnSetSmpteStart() assumes we are already generating SMPTE, when we do a jam sync we have already sent bit 0
    // set us to bit 1
    g_ucSmpteCurrentBitNumber    = 0;
    
    // kw v - dropframe
    g_bDropFrameTopOfMinute = false;
    g_bDropFrameFramesDropped = false;
    g_uiSmpteTosVariance = 0;
    g_uiSmpteTosCount = 0;
    g_uiSmpteTosSum = 0;
    g_uiSmpteTosGreatest = 0;
    g_uiSmpteTosLeast = 0;
    g_iSmpteTosPrevious = 0;
    
    //g_uiDF_err = 0;
   // g_bDF_add_skew = false;
    //g_bDF_more_skew = false;
    //g_bDF_less_skew = false;
    
    //g_uiDF_Skew_Modulus = 40;
    //g_uiDF_Skew_Modulus_2 = 40;
    //g_uiDF_Skew_Modulus_3 = 40;
    
    //gb_DropFrameTimedStart = false;
    //gb_DropFrameSync = false;
    g_ucEdgeNumber = 0;
    g_bDF_Sync_Now = false;
    
    g_bDropFrameJam = true; // Causes df tos to happen at right time on power up or when user changes tc format.
    // end kw

  //  g_uiStartTimeCodeOutput = START_SMPTE; // Start the SMPTE output at the next top of second.
}





//*****************************************************************************
//      Name:           fnSmpteGeneratorStart_2()
//
//      Description:    Start the channel 2 SMPTE output synchronized to the top of second.
//                      NOTE::: this functions can only be called near the half-second
//                              to make sure we are not near the top of second.
//
//                      This is called from StartTcgOutput() at system start, 
//                      when tc config has changed, when jam syncing.
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorStart_2(void)
{
    #ifdef _DEBUG_TCG
    debugPrintf("Waiting to start SMPTE 2\n");
    #endif
    
    debugPrintf("fnSmpteGeneratorStart_2\n");

	// SMPTE bits always start with a high, get ready by setting the port low
    // If we are currently generating SMPTE, this will stop the time code output      
    
    // Sets Flex timer 3, channel 2, FTM3_CH2
    fnFtm3Ch2SetOutputSmpte_2(1); // this will force the output high
// return; ch 1 comes up with this.
    
    memset(g_ulIsrFtm3_LastDiff_2, 0, sizeof(g_ulIsrFtm3_LastDiff_2));

    // Heavy work. Make the frame. Get current time, apply TZ, DST, put into buffer.
    fnTcgSysSetNextDateTime(1); // parm is thread # to get to 2nd smpte config
// return; ch 1 comes up with this.
    g_bDatatBit_Smpte_2              = false;  // the first bit is always a zero

    g_ucSmpteCurrentByte_2           = 0;
    g_ucSmpteCurrentBitMask_2        = 0x01;

    g_bIsHalfBit_2   			   = true;  // the next half bit is not the start of the bit

    // This will cause fnSetSmpteStart() to start a new frame
    g_ucSmpteCurrentBitNumber_2 = MAX_SMPTE_BITS + 2;
    g_ucSmpteFrame_Counter_2    = g_ucSmpteTcgMaxFrameValue_2 + 2;

    // the first bit after a jam sync is always a zero, we do not toggle at the half bit.
    fnSetSmpteStart_2(); // make the frame - put in data.

    // fnSetSmpteStart() assumes we are already generating SMPTE, when we do a jam sync we have already sent bit 0
    // set us to bit 1
    g_ucSmpteCurrentBitNumber_2    = 0;
    
    g_ucEdgeNumber_2 = 0;
    g_bDF_Sync_Now_2 = false;
    
    g_bDropFrameJam_2 = true; // Causes df tos to happen at right time on power up or when user changes tc format.
//return; //ch 1 comes up with this.
  //  g_uiStartTimeCodeOutput = START_SMPTE_2; // Start the channel 2 SMPTE output at the next top of second.
    
    g_uiStartTimeCodeOutput = START_SMPTE_RC1000; 
        
} // end fnSmpteGeneratorStart_2








//*****************************************************************************
//      Name:           fnSmpteGeneratorSetNextDateTime()
//
//      Description:    Put data into next-second-buffer: time, Leitch date, encoding.
//                      Called from TcgSysSetNextDateTime() and when config is set.
//
//		Notes:			Not blocking
//                      Channel parm is 1 or 2, for smpte output channel 1 or 2, in Rc1000.
//                      Passing a define instead of pointer because easier to see define when debugging.
//
//		MP safe:		Unknown
// 
//void fnSmpteGeneratorSetNextDateTime(SDateTimeUsec *psrDateTime)
void fnSmpteGeneratorSetNextDateTime(SDateTimeUsec *psrDateTime, INT i_Smpte_Channel) // kw rc1000
{   
    // kw rc1000 start
    BOOLEAN b_Err;
    
    switch(i_Smpte_Channel)
    {
    case SMPTE_CHANNEL_1:
        b_Err = fnLockSmpte("SMPTE_CH_1");
        if ( !b_Err )
        {
            gp_SmpteNextSecondBuffer = g_aucSmpteNextSecondBuffer;
            gp_SmpteCurrentConfig = &g_srSmpteCurrentConfig;
            gp_Tcg_Config = &g_srTcgConfig;
            fnUnlockSmpte("SMPTE_CH_1");
        }
        else
        {
            debugPrintf("!!!SMPTE 1 lock error.\n");
        }
        break;
    case SMPTE_CHANNEL_2:
        b_Err = fnLockSmpte("SMPTE_CH_2");
        if ( !b_Err )
        {
            gp_SmpteNextSecondBuffer = g_aucSmpteNextSecondBuffer_2;
            gp_SmpteCurrentConfig = &g_srSmpteCurrentConfig_2;
            gp_Tcg_Config = &g_srTcgConfig_2;
            fnUnlockSmpte("SMPTE_CH_2");
        }
        else
        {
            debugPrintf("!!!SMPTE 2 lock error.\n");
        }
        break;
    default: // use ch 1
        b_Err = fnLockSmpte("SMPTE_CH_1");
        if ( !b_Err )
        {
            gp_SmpteNextSecondBuffer = g_aucSmpteNextSecondBuffer;
            gp_SmpteCurrentConfig = &g_srSmpteCurrentConfig;
            gp_Tcg_Config = &g_srTcgConfig;
            fnUnlockSmpte("SMPTE_CH_1");
        }
        else
        {
            debugPrintf("!!!SMPTE 1 lock error.\n");
        }
        break;
    }
    // kw rc1000 end

    memset( gp_SmpteNextSecondBuffer, 0, sizeof(g_aucSmpteNextSecondBuffer));

	gp_SmpteNextSecondBuffer[SMPTE_BYTE_SYNC_WORD_MSB] = SMPTE_SYNC_WORD_MSB;//set sync bits
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_SYNC_WORD_LSB] = SMPTE_SYNC_WORD_LSB;

    // Encode time into NextSecondBuffer
    fnSmpteGeneratorSetTime(psrDateTime);
	//fnSmpteGeneratorSetTime(psrDateTime, i_Smpte_Channel); // below
    
    // kw v - dropframe - set flag if whole minute so that frames are NOT dropped.
    // Drop happens at every top of minute, AND when minutes is NOT a multiple of 10.
    if ((psrDateTime->m_srTime.m_ucSecond == 0) && (psrDateTime->m_srTime.m_ucMinute % 10))
    {
        switch(i_Smpte_Channel)
        {
        case SMPTE_CHANNEL_1:
            g_bDropFrameTopOfMinute = true;
            break;
        case SMPTE_CHANNEL_2:
            g_bDropFrameTopOfMinute_2 = true; // kw rc1000
            break;
        }
    }
    // end kw
    

	//switch(g_srSmpteCurrentConfig.m_ucSmpteExtraEncoding)
    switch(gp_SmpteCurrentConfig->m_ucSmpteExtraEncoding)
	{
		case SMPTE_ENCODING_TCG_LEITCH_DATE:
			fnSmpteGeneratorSetLeitchDate(psrDateTime);
			//if ( g_srTcgConfig.m_srSmpteConfig.m_ucSmpteMasterclockControlBits & SMPTE_MASTERCLOCK_CONTROL_BITS )
			//{
			//	fnSmpteGeneratorSetMasterclockControlBits(); // FreeWheeling, Leap Second, DST
			//}
			break;

		case SMPTE_ENCODING_TCG_S309M_MMDDYY_TIME_ZONE_INCLUDED:
			fnSmpteGeneratorSet309M_DateMMDDYY(psrDateTime);
			fnSmpteGeneratorSet309M_TimeZone(i_Smpte_Channel);
			fnSmpteGeneratorSet309M_BinaryGroupFlags();
			break;

        case SMPTE_ENCODING_TCG_S309M_MMDDYY_TIME_ZONE_NOT_INCLUDED:
            fnSmpteGeneratorSet309M_DateMMDDYY(psrDateTime);
            //fnSmpteGeneratorSet309M_TimeZone(); not needed, time zone always zero
            fnSmpteGeneratorSet309M_BinaryGroupFlags();
            break;

		case SMPTE_ENCODING_TCG_S309M_MJD_TIME_ZONE_INCLUDED:
			fnSmpteGeneratorSet309M_DateMJD(psrDateTime);
			fnSmpteGeneratorSet309M_TimeZone(i_Smpte_Channel);
			fnSmpteGeneratorSet309M_BinaryGroupFlags();
			break;

        case SMPTE_ENCODING_TCG_S309M_MJD_TIME_ZONE_NOT_INCLUDED:
            fnSmpteGeneratorSet309M_DateMJD(psrDateTime);
            //fnSmpteGeneratorSet309M_TimeZone(); not needed, time zone always zero
            fnSmpteGeneratorSet309M_BinaryGroupFlags();
            break;

		case SMPTE_ENCODING_TCG_NONE:
			//if ( g_srTcgConfig.m_srSmpteConfig.m_ucSmpteMasterclockControlBits & SMPTE_MASTERCLOCK_CONTROL_BITS )
			//{
			//	fnSmpteGeneratorSetMasterclockControlBits(); // FreeWheeling, Leap Second, DST
			//}
			break;
	}
}


//*****************************************************************************
//      Name:           fnSmpteGeneratorSetTime()
//
//      Description:    Put time into NextSecondBuffer for next smpte frame.
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSetTime(SDateTimeUsec *psrDateTime)
//void fnSmpteGeneratorSetTime(SDateTimeUsec *psrDateTime, INT i_Smpte_Channel) // kw rc1000
{
	UINT8 ucTempValue;
	//UINT16 sDaysOfYear;
    
    // kw rc1000 use gp_SmpteNextSecondBuffer

	//Sec
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucSecond);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_SECOND_ONES] |= (ucTempValue) & 0x0F;       //units
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_SECOND_TENS] |= (ucTempValue >> 4) & 0x07;  //tens

	//Min
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucMinute);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_MINUTE_ONES] |= (ucTempValue) & 0x0F;       //units
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_MINUTE_TENS] |= (ucTempValue >> 4) & 0x07;  //tens

	//Hour
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucHour);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_HOUR_ONES] |= (ucTempValue) & 0x0F;         //units
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_HOUR_TENS] |= (ucTempValue >> 4) & 0x03;    //tens
    
    /*
    //Sec
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucSecond);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_SECOND_ONES] |= (ucTempValue) & 0x0F;       //units
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_SECOND_TENS] |= (ucTempValue >> 4) & 0x07;  //tens

	//Min
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucMinute);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_MINUTE_ONES] |= (ucTempValue) & 0x0F;       //units
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_MINUTE_TENS] |= (ucTempValue >> 4) & 0x07;  //tens

	//Hour
	ucTempValue = bin_to_BCD(psrDateTime->m_srTime.m_ucHour);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_HOUR_ONES] |= (ucTempValue) & 0x0F;         //units
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_HOUR_TENS] |= (ucTempValue >> 4) & 0x03;    //tens
    */
}



//*****************************************************************************
//      Name:           fnSmpteGeneratorSet309M_TimeZone()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSet309M_TimeZone(INT i_Smpte_Channel)
{
	unsigned long   ulSeconds;
	BOOLEAN bBiasIsNegative;

	UINT8 ucHour, ucMinute;
	UINT8 ucCode;

	// In the future, the offset can be in seconds, and frames or fractions of a second.
	// For now, we only use whole seconds.

    // Convert the offset to hours, minutes, seconds.
    //ulSeconds = g_srTcgConfig.m_srTcgMiscConfig.ulTcgSecondsOffset;
    ulSeconds = gp_Tcg_Config->m_srTcgMiscConfig.ulTcgSecondsOffset;
    

	if ( ulSeconds )
	{
 /*
        switch(i_Smpte_Channel) // kw rc1000
        {
        case SMPTE_CHANNEL_1:
            if ( g_srTcgConfig.m_srTcgMiscConfig.ucTcgTimeOffsetSign )
            {
                bBiasIsNegative = true;
            } else
            {
                bBiasIsNegative = false;
            }
            break;
        case SMPTE_CHANNEL_2:
            if ( g_srTcgConfig_2.m_srTcgMiscConfig.ucTcgTimeOffsetSign )
            {
                bBiasIsNegative = true;
            } else
            {
                bBiasIsNegative = false;
            }
            break;
        default:
            if ( g_srTcgConfig.m_srTcgMiscConfig.ucTcgTimeOffsetSign )
            {
                bBiasIsNegative = true;
            } else
            {
                bBiasIsNegative = false;
            }
            break;
        }
*/
        
        //if ( g_srTcgConfig.m_srTcgMiscConfig.ucTcgTimeOffsetSign )
        if ( gp_Tcg_Config->m_srTcgMiscConfig.ucTcgTimeOffsetSign )
        {
            bBiasIsNegative = true;
        } else
        {
            bBiasIsNegative = false;
        }
        
		ucHour     = (UINT8)(ulSeconds / 3600UL);
		ulSeconds -= ((unsigned long)ucHour * 3600UL);

		ucMinute   = (UINT8)(ulSeconds / 60UL);
		ulSeconds -= ((unsigned long)ucMinute * 60UL);

		//ucSecond   = (UINT8)ulSeconds;

		// convert the hours, minutes, seconds into the SMPTE 309M-1999 code
		// See the SMPTE 309M-1999 Table 2

		ucCode = 0;  // UTC

		switch ( ucMinute )
		{
			case 0:
				// minutes == 0
				// check for codes 01-25
				if ((ucHour >= 1) && (ucHour <= 13))
				{
					// the code is in this table
					if ( bBiasIsNegative )
					{
						ucCode = gucSmpteOffsetTable1[ucHour];
					} else
					{
						ucCode = gucSmpteOffsetTable2[ucHour];
					}

				} else
				{
					// not a code in the table
					ucCode = 0x38; // User defined time Offset
				}
				break;

			case 30:
				// minutes == 30
				if (ucHour <= 11)
				{
					if ( bBiasIsNegative )
					{
						ucCode = gucSmpteOffsetTable3[ucHour];
					} else
					{
						ucCode = gucSmpteOffsetTable4[ucHour];
					}

				} else
				{
					// not a code in the table
					ucCode = 0x38; // User defined time Offset
				}
				break;

			case 45:
				// minutes == 45
				if ((ucHour == 12) && !bBiasIsNegative)
				{
					ucCode = 0x32;

				} else
				{
					// not a code in the table
					ucCode = 0x38; // User defined time Offset
				}
				break;

			default:
				// not a code in the table
				ucCode = 0x38; // User defined time Offset
				break;
		}

		// the ucCode gets placed into binary group 7 and 8
		// See SMPTE 309M-1999 Table 1
	//	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_TZ_TYPE_CONTROL1]  |= ((ucCode<<4) & 0xF0);	 // binary group 7

	//	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_TZ_TYPE_CONTROL2]  |= (ucCode & 0x30);	     // binary group 8
        
        // kw rc1000 - buf for ch 1 or 2
        gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_TZ_TYPE_CONTROL1]  |= ((ucCode<<4) & 0xF0);	 // binary group 7

		gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_TZ_TYPE_CONTROL2]  |= (ucCode & 0x30);	     // binary group 8
	}
}






//*****************************************************************************
//      Name:           fnSmpteGeneratorSet309M_DateMMDDYY()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSet309M_DateMMDDYY(SDateTimeUsec *psrDateTime)
{
	UINT8 ucBcdYear;
	UINT8 ucBcdMonth;
    UINT8 ucBcdDate;

	ucBcdYear  = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_sYear - CURRENT_CENTURY));
	ucBcdMonth = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_ucMonth));
	ucBcdDate  = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_ucDate));
/*
	//set the date  shift bits into correct position and mask out irrelevent bits
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_DATE_ONES]  |= ((ucBcdDate<<4) & 0xF0);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_DATE_TENS]  |= (ucBcdDate & 0x30);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MONTH_ONES] |= ((ucBcdMonth<<4) & 0xF0);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MONTH_TENS] |= (ucBcdMonth & 0x10);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_YEAR_ONES]  |= ((ucBcdYear<<4) & 0xF0);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_YEAR_TENS]  |= (ucBcdYear & 0xF0);

    //clear MJD flag bit, this indicates the date is in MMDDYY format
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] &= (~0x80);
*/
    
        //set the date  shift bits into correct position and mask out irrelevent bits
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_DATE_ONES]  |= ((ucBcdDate<<4) & 0xF0);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_DATE_TENS]  |= (ucBcdDate & 0x30);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MONTH_ONES] |= ((ucBcdMonth<<4) & 0xF0);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MONTH_TENS] |= (ucBcdMonth & 0x10);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_YEAR_ONES]  |= ((ucBcdYear<<4) & 0xF0);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_YEAR_TENS]  |= (ucBcdYear & 0xF0);

    //clear MJD flag bit, this indicates the date is in MMDDYY format
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] &= (~0x80);
}



//*****************************************************************************
//      Name:           fnSmpteGeneratorSet309M_DateMJD()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSet309M_DateMJD(SDateTimeUsec *psrDateTime)
{
	UINT8 ucMonth;
	unsigned long ulDay, ulDayTemp;
	UINT8 ucDay_BCD;
	short sNumYears;

	// convert the current date to MJD

	// Per the SMPTE 309M-1999 standard, 1/1/1995 == MJD 49718
	// We'll use 1996 as our baseline since it is a leap year.
	// So, add 365 to the 49718.
	sNumYears = psrDateTime->m_srDate.m_sYear-1996;
	ulDay = (49718ul + 365ul) + (unsigned long)(sNumYears*365);

	// add in leap years
	ulDay += (sNumYears>>2);   // Note: this adds in the leap day for years up to and including the current year
	                           //       since our base year was 1996

	ucMonth = psrDateTime->m_srDate.m_ucMonth-1; // subtract one, we haven't completed the month yet.

	// days in this month
	ulDay += psrDateTime->m_srDate.m_ucDate;

	// add in the days of this year
	// Is this a leap year?
	if ( !(psrDateTime->m_srDate.m_sYear & 0x03) )
	{
		// Yes, this is a leap year
		// Subtract 1, we may not be past Feb 28
		ulDay -= 1;

		// The usDaysMonthElapsedLeap will add the leap day back in if needed.
		ulDay += usDaysMonthElapsedLeap[ucMonth];
	} else
	{
		ulDay += usDaysMonthElapsed[ucMonth];
	}

	//set the date  shift bits into correct position and mask out irrelevent bits
	//first change from binary bumber to BCD, the macro bin_to_BCD will not work here
	//it requires a smaller number to convert

	//the HUNDRED THOUSANDS digit is always zero until the year 2131.
	//ulDayTemp = ulDay/100000ul;
	//ucDay_BCD = (UINT8)ulDay;

	//Set the HUNDRED THOUSANDS digit, it is already zero, don't need to do this until year 2131
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_HUNTHOUSANDS] |= (ucDay_BCD & 0xF0);

	//find the TEN THOUSANDS digit
	//ulDay     = ulDay - (ulDayTemp*100000ul); // again, no hundred thousands until 2131
	ulDayTemp = ulDay / 10000ul;
	ucDay_BCD = (UINT8)ulDayTemp;
	ucDay_BCD <<= 4;

	//Set the TEN THOUSANDS digit
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_TENTHOUSANDS] |= (ucDay_BCD & 0xF0);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_TENTHOUSANDS] |= (ucDay_BCD & 0xF0); // kw rc1000

	//find the THOUSANDS digit
	ulDay     = ulDay - (ulDayTemp*10000ul);
	ulDayTemp = ulDay / 1000ul;
	ucDay_BCD = (UINT8)ulDayTemp;
	ucDay_BCD <<= 4;

	//Set the THOUSANDS digit
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_THOUSANDS] |= (ucDay_BCD & 0xF0);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_THOUSANDS] |= (ucDay_BCD & 0xF0); // kw rc1000

	//find the HUNDREDS digit
	ulDay     = ulDay - (ulDayTemp*1000ul);
	ulDayTemp = ulDay / 100ul;
	ucDay_BCD = (UINT8)ulDayTemp;
	ucDay_BCD <<= 4;

	//Set the HUNDREDS digit
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_HUNDREDS] |= (ucDay_BCD & 0xF0);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_HUNDREDS] |= (ucDay_BCD & 0xF0); // kw rc1000

	//find the TENS digit
	ulDay     = ulDay - (ulDayTemp*100ul);
	ulDayTemp = ulDay / 10ul;
	ucDay_BCD = (UINT8)ulDayTemp;
	ucDay_BCD <<= 4;

	//Set the TENS digit
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_TENS] |= (ucDay_BCD & 0xF0);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_TENS] |= (ucDay_BCD & 0xF0); // kw rc1000

	//find the ONES digit
	ulDay     = ulDay - (ulDayTemp*10ul);
	ucDay_BCD = (UINT8)ulDay;
	ucDay_BCD <<= 4;

	//Set the ONES digit
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_ONES] |= (ucDay_BCD & 0xF0);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_309M_MJD_ONES] |= (ucDay_BCD & 0xF0); // kw rc1000

    //set MJD flag bit, this indicates the date is in Modified Julian Date, MJD format
    //g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x80;
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x80; // kw rc1000
}








//*****************************************************************************
//      Name:           fnSmpteGeneratorSet309M_BinaryGroupFlags()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSet309M_BinaryGroupFlags(void)
{
    UINT8 ucCurrentSyncSource, ucCurrentSyncSourceStatus;

//
//
// The Binary Group Flag bits are for 309M.
// BGF0 - Binary Group Flag Bit 0 is bit 43
// BGF1 - Binary Group Flag Bit 1 is bit 58
// BGF2 - Binary Group Flag Bit 2 is bit 59
//
//  BGF2 BGF1 BGF0
//   1    0    0    unspecified, ie. not locked
//   1    1    0    precision clock, ie. locked to a reference
//
//

	//BGF2 bit 59, 8th binary group
	//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x08;
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x08; // kw rc1000

	// by default all bits are set to zero
	// we only need to set the one's
    ucCurrentSyncSource       = fnSysSyncGetCurrentSyncSource();
    ucCurrentSyncSourceStatus = fnSysSyncGetSyncSourceStatus(ucCurrentSyncSource);

    if (ucCurrentSyncSource != REFERENCE_FAILURE)
    {
        if ( ucCurrentSyncSourceStatus != REFERENCE_TIME_NOT_LOCKED)
    	{
    		// if locked, set the bit for BGF1
    		//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x04;
            gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP8] |= 0x04; // kw rc1000
    	}
    }

	//BGF0 is always zero
}





//*****************************************************************************
//      Name:           fnSmpteGeneratorSetLeitchDate()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSetLeitchDate(SDateTimeUsec *psrDateTime)
{
	UINT8 ucYear;
	UINT8 ucMonth;
    UINT8 ucDate;
    //change binary number to BCD for transmission
	ucYear  = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_sYear - CURRENT_CENTURY));
    ucMonth = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_ucMonth));
	ucDate  = bin_to_BCD((UINT8)(psrDateTime->m_srDate.m_ucDate));

	//set the date  shift bits into correct position and mask out irrelevent bits
/*
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_DATE_ONES]  |= ((ucDate<<4)  & 0xF0 );//((ucDate)& 0x0F );//
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_DATE_TENS]  |=  (ucDate      & 0x30);
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_MONTH_ONES] |= ((ucMonth<<4) & 0xF0 );//
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_MONTH_TENS] |= ((ucMonth<<2) & 0x40);
    g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_YEAR_ONES]  |= ((ucYear<<4)  & 0xF0 );//
	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_YEAR_TENS]  |=  (ucYear      & 0xF0);
*/
    // kw rc1000
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_DATE_ONES]  |= ((ucDate<<4)  & 0xF0 );//((ucDate)& 0x0F );//
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_DATE_TENS]  |=  (ucDate      & 0x30);
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_MONTH_ONES] |= ((ucMonth<<4) & 0xF0 );//
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_MONTH_TENS] |= ((ucMonth<<2) & 0x40);
    gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_YEAR_ONES]  |= ((ucYear<<4)  & 0xF0 );//
	gp_SmpteNextSecondBuffer[SMPTE_BYTE_LEITCH_YEAR_TENS]  |=  (ucYear      & 0xF0);
}





//*****************************************************************************
//      Name:           fnSmpteGeneratorSetMasterclockControlBits()
//
//      Description:
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorSetMasterclockControlBits(void)
{

    UINT8 ucCurrentSyncSource, ucCurrentSyncSourceStatus;

//
//
// The Masterclock Control Bits are in the 1st User Group (1st Binary Group)
// Freewheeling flag is SMPTE bit 4; 1=Freewheeling, 0=normal operation (ie. 1=not locked to reference, 0=locked to reference)
// Leap Second flag is SMPTE bit 6; 1=leap second pending, up to 60 seconds prior to the leap second insertion
//                                                   this bit gets set, see IRIG1344 specification
//                            0=no leap second pending
//                            We do not ghave a sign indicating if the leap second is positive or negative.
//                            Up to this time, all leap seconds have been positive, ie. add a second with a value of 60
//
// Daylight time flag is SMPTE bit 7; 1=daylight savings time in progress
//                              0=standard time
//

	// by default all bits are set to zero
	// we only need to set the one's

	// 1st Binary group, SMPTE bit 4
    // by default all bits are set to zero
    // we only need to set the one's
    ucCurrentSyncSource       = fnSysSyncGetCurrentSyncSource();
    ucCurrentSyncSourceStatus = fnSysSyncGetSyncSourceStatus(ucCurrentSyncSource);

    if (ucCurrentSyncSource != REFERENCE_FAILURE)
    {
        if ( ucCurrentSyncSourceStatus != REFERENCE_TIME_NOT_LOCKED)
        {
		// if locked, set the bit
		//g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x10;        }
        gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x10;        } // kw rc1000
    }


	// 1st Binary group, SMPTE bit 6
	//if (g_srTcgConfig.m_srTcgMiscConfig.ucTcgLeapSecondPending)
	//{
		// if locked, set the bit
	//	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x40;
	//}
    
    // kw rc1000
    if (gp_Tcg_Config->m_srTcgMiscConfig.ucTcgLeapSecondPending)
	{
		// if locked, set the bit
		gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x40;
	}

	// 1st Binary group, SMPTE bit 7
	//if ((g_srTcgConfig.m_srTcgMiscConfig.ucTcgDaylightSavingPending == 1) || (g_srTcgConfig.m_srTcgMiscConfig.ucTcgDaylightSavingActive))
	//{
	//	g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x80;
	//}
    
    // kw rc1000
    if ((gp_Tcg_Config->m_srTcgMiscConfig.ucTcgDaylightSavingPending == 1) || (gp_Tcg_Config->m_srTcgMiscConfig.ucTcgDaylightSavingActive))
	{
		gp_SmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] |= 0x80;
	}
}




//*****************************************************************************
//      Name:           fnSmpteGeneratorCopyBuffer()
//
//      Description:    Copies the the current SMPTE output buffer.
//                      Called only from ContolCmdStatsMCR1000Tcg
//
//		Notes:			Not blocking
//
//		MP safe:		Unknown
//
void fnSmpteGeneratorCopyBuffer(UINT8 *psrTcgTc, UINT8 length)
{
    DI();
	memcpy(psrTcgTc, g_aucSmpteCurrentBuffer, length);
    EI();
}





//*****************************************************************************
//      Name:           fnSmpteGeneratorTimerIsr()
//
//      Description:
//
//		Notes:			Isr
//						Timer1 is a set to 16bit mode and counts up and cleared
//						when timer1 matches matchA
//                      Time code signal is the TC_Carrier output (OC1A) modulated
//                      by the TC_Modulator (OC1B output. Each bit sent consist of
//                      10 counts of the Bit Timing Count (highs and lows)
//                      Kinetis: FTM3_CH1 SMPTE out, PTD1, pin 128. See flextimer.c
//
//                      kw q: when and how often is this run? Every half bit.
//
//		MP safe:		Unknown
//
#pragma optimize=speed
void fnSmpteGeneratorTimerIsr(BOOLEAN bIsRolloverAdded)
{

    SMPTE_SKEWING *psrSmpteSkewing; // using pointers is faster

	//UINT16 usCurrentMatchValue;
    UINT    uiSmpteMatchValue;
    UINT    uiTempTicks;

    BOOLEAN bIsNextOutputHigh;
    BOOLEAN bIsSmpteTopOfSecond;
    //BOOLEAN bIsNewFrame; // kw v dropframe
    
    UINT uiCurrentTicks;

    INT iSkewDiff;

    static UINT uiIsApply;
    
    g_ucEdgeNumber++;
    if (g_ucEdgeNumber >= DF_SKEW_TEMP_MAX)
        g_ucEdgeNumber = 0;
    

    bIsSmpteTopOfSecond = false;
    psrSmpteSkewing     = &g_srSmpteGenSkewing;
	// g_usSmpteMatchValue is constant for every run of this func. It is set by type of time code being generated.
    uiSmpteMatchValue   = g_usSmpteMatchValue; // global depends on type of smpte. = freq div by num-half-bits.
    bIsNextOutputHigh   = g_bSmpteNextOutputStateIsHigh;

    // roughly in the middle of a second, update the skewing
    if ((g_ucSmpteFrame_Counter == 12) && (g_ucSmpteCurrentBitNumber == 4) && (!g_bIsHalfBit)
        && (!(g_srRC600Control.ucOpMode & (SMPTE_COUNT_MASK | SMPTE_INIT_MASK))))
    {
        // kw comment: this to limit calcs to once per second?
      
        if (psrSmpteSkewing->m_bSmpteTosCaptured) // This is true when pps generated at system tos, so cannot be used for dropframe.
        {
            psrSmpteSkewing->m_bSmpteTosCaptured = false;
            
            if (!g_bDropFrameGenerating) // kw v dropframe - see idle thread for jam of df.
            {
                // If we are within three frames of the reference then skewing is allowed.
                // kw comment: this 'if' is for the opposite, when we need to jam sync.
                if ((psrSmpteSkewing->m_uiSmpteFrameAtTopOfSecond > 3) && (psrSmpteSkewing->m_uiSmpteFrameAtTopOfSecond < (g_ucSmpteTcgMaxFrameValue-3)))
                {
                    LED_YELLOW_NTP_LONG_BLINK_ISR();
                    LED_GREEN_LONG_BLINK_ISR();

                    // We need to do a jam sync
                    // This is done outside of an interrupt routine.
                    g_srMsecThread.uiStartTimeCodePending = START_TIME_CODE_PENDING_SMPTE_1;
                    g_uiStartTimeCodeData1 = psrSmpteSkewing->m_uiSmpteFrameAtTopOfSecond;
                    g_uiStartTimeCodeData2 = psrSmpteSkewing->m_uiSmpteBitCountAtTopOfSecond;
                    return;
                }
            }

            // kw comment: don't see why "we are within one bit of being correct" below.
            //
            // We know we are within one bit of being correct.
            // We know that the top of second time comes from the system, therefore
            // we cannot have an error in time of more than one bit.
            //

            // g_ullIsrFtm3_TC_AtTopOfSecond contains the value of FTM3_CH3 at the system's top of second
            // We compare this to the value of the time code generators counter value at top of second.
            // If FTM3 is running at 10 Mhz, 16 bits rolls over in 6.5536 milleseconds.
            // If FTM3 is running at 15 Mhz, 16 bits rolls over in 4.3691 milleseconds.
            // The maximum we will allow time code to skew is 2 milleseconds per second.
            // We know timecode was started within a few microseconds of top of second.
            // As long as time code can skew at a rate greater than the system skewing,
            // time code can never be more than 2 milleseconds from system top of second
            // unless a jam sync occurred. If this happens, we will be more than
            // 2 bits off and time code will also do a jam sync.
            
            // def of g_ullIsrFtm3_TC_MatchAtTopOfSecond is in flextimer.c.
            // It is used because the timer for smpte, FTM3_CH1, is free running, so it is not set back to 0 on match. 
            // Thus we cannot just save a match value once and then repeatedly compare the timer to it.
            // Instead we have to update the match value, repeatedly adding to it to "stay ahead" of the timer. 
            // So we add to g_ullIsrFtm3_TC_MatchAtTopOfSecond.
            // It is set to 0 in flextimer.c and below, for smpte. It's also used for irig.
            
            uiIsApply++;
            if (uiIsApply > 1)
            {
                uiIsApply = 0;
            }
            if (!uiIsApply)
            {
                // If we get a large difference, wait three seconds before we use it
                // This is a crude filter/hack. We were observing times when there was a difference of 50000 ticks
                // for no apparent reason. Have not found the root cause. This works around the issue.
                // I DON'T LIKE THIS SOLUTION!!!
                if (g_ullIsrFtm3_TC_AtTopOfSecond >= g_ullIsrFtm3_TC_MatchAtTopOfSecond)
                {
                    iSkewDiff = (INT)(g_ullIsrFtm3_TC_AtTopOfSecond - g_ullIsrFtm3_TC_MatchAtTopOfSecond);
                } else
                {
                    iSkewDiff = (INT)(g_ullIsrFtm3_TC_MatchAtTopOfSecond - g_ullIsrFtm3_TC_AtTopOfSecond);
                }

                g_ulIsrFtm3_LastDiff[3] = iSkewDiff;

                if (iSkewDiff >= 5000)
                {
                    if ((g_ulIsrFtm3_LastDiff[0] < 5000) && (g_ulIsrFtm3_LastDiff[1] < 5000) && (g_ulIsrFtm3_LastDiff[2] < 5000))
                    {
                        iSkewDiff = 0;
                    }
                }

                g_ulIsrFtm3_LastDiff[0] = g_ulIsrFtm3_LastDiff[1];
                g_ulIsrFtm3_LastDiff[1] = g_ulIsrFtm3_LastDiff[2];
                g_ulIsrFtm3_LastDiff[2] = g_ulIsrFtm3_LastDiff[3];

                if (g_ullIsrFtm3_TC_AtTopOfSecond >= g_ullIsrFtm3_TC_MatchAtTopOfSecond)
                {
                    // time code is behind system time, need to
                    // speed up by adding ticks
                    // kw comment: is this really the # of ticks? Isn't it just the dif between counts?
                    psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks = iSkewDiff;
                } else
                {
                    // time code is ahead of system time, need to
                    // slow down up by subtracting ticks
                    psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks  = iSkewDiff;
                    psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks  = -psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks;
                }
            } else
            {
                // We only skew to the reference every other second
                psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks = 0;
            }

            // Since SMPTE timing does not divide into our clock ticks, we must add in the number of ticks we lost.
            psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks += g_usSmpteModulatorSkewPreload; // rhs is always 0, so no change.

            // two edges per bit. we are skewing on every edge
            psrSmpteSkewing->m_iNextMcrSmpteSkewPerEdge = (psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks/g_usSmpteTotalNumHalfBits) + 1;

            // kw comment: above says limit is 2ms/s
            // limit skewing to 10 msec/second
            if (psrSmpteSkewing->m_iNextMcrSmpteSkewPerEdge > (g_uiSmpteMaxSkewPerBit/2)) // two edges per bit
            {
                psrSmpteSkewing->m_iNextMcrSmpteSkewPerEdge = g_uiSmpteMaxSkewPerBit/2;
            }
            
            
            // When dropframe is being generated its TOS never aligns with system TOS
            // so the skewing scheme for df is to NOT use the above calcs that derive from SMPTE TOS,
            // but instead to use the skew that the system used for its TOS.
            
            
			if (g_bDropFrameGenerating)
            {
                psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks = 0;
                psrSmpteSkewing->m_iNextMcrSmpteSkewPerEdge = 0;
                
                // The skewing scheme for other tc is not workable for dropframe.
                // See the idle thread.
            }

////kjf xxx yyy zzz disable skewing
//if (iSkewIndex>=100)
//{
//    iSkewIndex = 0;
//}
//g_iSaveSkew[iSkewIndex] = psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks;
//iSkewIndex++;
            
        } // end if smpte tos captured.
    } // end update skewing in middle of second.

		//If true, we are at the half bit setting up for the start of the bit
		if(g_bIsHalfBit)
		{
			g_bIsHalfBit = false;
			g_ucSmpteCurrentBitNumber++;

            
            // kw v dropframe - df has to be started at the right ms.
            if (g_bDropFrameGenerating && g_bDF_Sync_Now) // if startframe has been calculated. 
            {
                g_bDF_Sync_Now = false; 
                g_ucSmpteFrame_Counter = g_ucSmpteTcgMaxFrameValue; // force SetSmpteStart() to return true.
                g_ucSmpteCurrentBitNumber = MAX_SMPTE_BITS; 
            }
            // end kw
                
            
            
			// If at end of SMPTE frame (Maximum bits) start next frame.
            // This causes skewing #s calculated above ('next') to be put into use.
            // Why wait until new second to do this?
            
			if(g_ucSmpteCurrentBitNumber >= MAX_SMPTE_BITS) // 80 bits per frame
			{

                // Fill in SmpteCurrentBuffer with next-second-buf or RC600 data, set bits in buf for df, polarity.
                bIsSmpteTopOfSecond = fnSetSmpteStart(); // next func. Work at start of every frame. TOS is when frames max.

                if (bIsSmpteTopOfSecond) // Following runs only at SMPTE TOS.
                {
                  
                    // Get the skewing values that were saved in the previous frame
                    psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks = psrSmpteSkewing->m_iNextMcrSmpteSkewClockTicks;
                    psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge    = psrSmpteSkewing->m_iNextMcrSmpteSkewPerEdge;
                    
                   if (g_bDropFrameGenerating)
                   {
                        // A channel can be set to either mode 'input capture' or 'output match'.
                        // If 'input compare' then its value FTMx_CyV cannot be set by code; it is set by the processor. 
                        // If 'output match' then code can put a number into FTMx_CyV,
                        // so the mode must be right before we do this.
                  
                        // Here we are at smpte dropframe tos, so send out a pulse, 
                        // the purpose being to capture the ticks per smpte second.
                        // (Ftm0 channel 6 was previously unused in generators.)
                        // Set match value of Ftm0 channel 6 to the current # of ticks in 1 ms plus the system's current milliseconds
                        // so that in 1 ms a pulse appears. The ftm0 interrupt routine has code added to clear the pulse.
                        
                        uiCurrentTicks = (UINT)FTM0_CNT;
                            // 1 ms of ticks + current ms of ticks. Why not just add reaload?
                        FTM0_C6V    = (UINT)g_uiFTM0_MsecReload + uiCurrentTicks; // Set the match value of the timer to now + 1 ms.
                        
                        // Clear the channel status of interrupts.
                        if (FTM0_C6SC & 0x80)
                        {
                            FTM0_C6SC &= ~0x80;
                        }

                        // The pulse is cleared in the FTM0 interrupt routine, channel 6, isrINT_FTM0 in flextimer.c.
                        // Channel 3: clear any pending interrupts, enable interrupts, set output on match
                        FTM0_C6SC = (UINT32)0x5CUL;     // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=1,DMA=0
                        
                        // Figure the current ticks so we can see how many have passed per 30 frames of df.
                        //g_ullDFticks2 = (UINT64)g_ullIsrFTM0_OverflowOffset + (UINT64)uiCurrentTicks;
                        //g_ullDF_MeasuredTicksPerSecond = (UINT64)g_ullDFticks2 - (UINT64)g_ullDFticks1; // used in idlethread.
                        //g_ullDFticks1 = g_ullDFticks2; // Save so we can calc next time.
                         
                        #ifdef _DEBUG_TCG_DROPFRAME
                            LED_YELLOW_NTP_ONE_BLINK_ISR(); // kw testing - visual aid.
                        #endif

                        // Calculate how far away from system tos the smpte tos is. Idle thread prints out.
                        g_ui_mS = g_uiCurrent_mS;
                        if (g_ui_mS <= 500) // smpte tos is after system tos
                        {
                            g_iSmpteTosPrevious = g_ui_mS;
                        }
                        else // smpte tos is before system's.
                        {
                            g_ui_mS = 1000 - g_ui_mS;
                  
                            g_iSmpteTosPrevious = 0 - g_ui_mS;
                        }
                        
                        // Here at drop frame TOS we copy the skew array from the temp that the idle thread filled in
                        // to the working array that is going to be added to the match value.
                        memcpy( g_aucDFskew, g_aucDFskew_temp, sizeof(g_aucDFskew));
  
                    } // end if df
					// end kw v dropframe

                } // end if smpte tos

			} // end bit# > max
			else// if not at end of frame set up for next bit
			{
				g_ucSmpteCurrentBitMask <<= 1;
				if(!g_ucSmpteCurrentBitMask)
				{
					g_ucSmpteCurrentBitMask = 0x01;
					g_ucSmpteCurrentByte++;
				}
			}
			g_bDatatBit_Smpte = (BOOLEAN)(g_aucSmpteCurrentBuffer[g_ucSmpteCurrentByte] & g_ucSmpteCurrentBitMask);

            // toggle output on match
            if (bIsNextOutputHigh) // was set to global at func start
            {
                bIsNextOutputHigh = false;
            } else
            {
                bIsNextOutputHigh = true;
            }
		}
		else // not at half bit
		{
			g_bIsHalfBit = true;
			if(g_bDatatBit_Smpte)
			{
				// If we are sending a '1' then toggle the output
				// which is the mode we are already set to
                if (bIsNextOutputHigh)
                {
                    bIsNextOutputHigh = false;
                } else
                {
                    bIsNextOutputHigh = true;
                }
			} else
			{
				// We are not sending a '1' so do not toggle the pin on the next match
				// Just generate an interrupt
			}
		}



    // Update the reload for any skewing needed. // kw: when is this done? Every frame? Why? 
	// Ticks and Edge are only calculated at TOS, and this code runs every frame, so dec ClockTicks by PerEdge every frame.
    // ClockTicks is the total skew to apply throughout the frame, and PerEdge is the amount to apply each time through here.
	// uiSmpteMatchValue starts out as the same for every call of this func,
	// so must repeatedly be changed by the amount to skew each edge.
	
    if (psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks) // if any ticks to skew
    {
        if (psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks >= 0) // if positive number of ticks
        {
			// kw comment: don't understand why test has ticks on left and edge on right.
			// PerEdge is ClockTicks/numhalfbits, so will PerEdge will always be less than ClockTicks.
            // Maybe reason is that PerEdge is forced down to a max value if too high.
            // Maybe also to avoid decrementing ClockTicks below 0.
			
            if (psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks >= psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge)
            {
                uiSmpteMatchValue                                 += (UINT)psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge;
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks -= psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge;
            } 
			else // The skews-per-edge have been used up, so just use up the remaining clock ticks.
            {
                uiSmpteMatchValue                                 += psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks;
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks  = 0;
            }

            if (psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks < 0)
            {
                // !!! cannot happen !!!
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks--;
            }
        } else // the clock ticks are negative
        {
            
            uiTempTicks = (UINT)(-psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks);

            if (uiTempTicks >= psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge)
            {
                uiSmpteMatchValue                                 -= (UINT)psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge;
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks += psrSmpteSkewing->m_iCurrentMcrSmpteSkewPerEdge;  // remember, we are adding to a negative number
            } else
            {
                uiSmpteMatchValue                                 -= uiTempTicks;
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks  = 0;
            }

            if (psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks > 0)
            {
                // !!! cannot happen !!!
                psrSmpteSkewing->m_iCurrentMcrSmpteSkewClockTicks++;
            }
        }
    } // end if any ticks to skew
    

    // kw v dropframe - the skew for df is in the elements of the array. 
    // Each element is 1 at most. Others are 0. Array was copied above at tos.
    if (g_bDropFrameGenerating)
    {   
        uiSmpteMatchValue += g_aucDFskew[g_ucEdgeNumber]; 
    }
    

    // Condition the output depending on what the next state should be.
    if (bIsNextOutputHigh != g_bSmpteNextOutputStateIsHigh)
    {

       // make sure the timer is stopped
       //FTM3_MODE |= (UINT32)0x05UL;                       // disable write protection, enable all register access
       //FTM3_SC    = (UINT32)((FTM3_SC & (UINT32)0xE7UL)); // no clock selected, disables FTM

       g_bSmpteNextOutputStateIsHigh = bIsNextOutputHigh;

       // guts
       if (bIsNextOutputHigh)
       {
           // Channel 1, enable interrupts, set the output on match
           FTM3_C1SC = (UINT32)0x5CUL;                        // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=1,res=0,DMA=0
       } else
       {
           // Channel 1, enable interrupts, clear the output on match
           FTM3_C1SC = (UINT32)0x58UL;                        // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=0,res=0,DMA=0
       }
    }

	// remember, FTM3_CH1 is free running, we do not reload on a match

    // If the next edge is time code top of second, save the counter value.
    if (bIsSmpteTopOfSecond)
    {
        if (bIsRolloverAdded)
        {
            if ((UINT)FTM3_C1V > (UINT)0x8000)
            {
                g_ullIsrFtm3_TC_MatchAtTopOfSecond = (g_ullIsrFTM3_OverflowOffset - (UINT64)0x10000) + (UINT64)FTM3_C1V + (UINT64)uiSmpteMatchValue; 
            } else
            {
                g_ullIsrFtm3_TC_MatchAtTopOfSecond = g_ullIsrFTM3_OverflowOffset + (UINT64)FTM3_C1V + (UINT64)uiSmpteMatchValue; 
            }
        } else
        {
            g_ullIsrFtm3_TC_MatchAtTopOfSecond = g_ullIsrFTM3_OverflowOffset + (UINT64)FTM3_C1V + (UINT64)uiSmpteMatchValue; 
        }
    }

    // as long as FTM3_C1SC is set to the correct mode, we can update the match value
    // without stopping the timer before setting it
    FTM3_C1V += uiSmpteMatchValue; // guts

    // is there a pending rollover we need to add?
    if (FTM3_CNT < 0x8000)
    {
        if (FTM3_SC & 0x80)
        {
            // there was an overflow, clear the overflow flag
            // and add the overflow offset
            FTM3_SC                     &= ~0x80;
            g_ullIsrFTM3_OverflowOffset += (UINT64)0x00010000;
        }
    }

} // end fnSmpteGeneratorTimerIsr





// kw rc1000 - handler for ftm 3, channel 2, smpte 2 output
//*****************************************************************************
//      Name:           fnSmpteGeneratorTimerIsr_2()
//
//      Description:
//
//		Notes:			Isr
//						Timer1 is a set to 16bit mode and counts up and cleared
//						when timer1 matches matchA
//                      Time code signal is the TC_Carrier output (OC1A) modulated
//                      by the TC_Modulator (OC1B output. Each bit sent consist of
//                      10 counts of the Bit Timing Count (highs and lows)
//                      Kinetis: FTM3_CH1 SMPTE out, PTD1, pin 128. See flextimer.c
//
//                      kw q: when and how often is this run? Every half bit.
//
//		MP safe:		Unknown
//
#pragma optimize=speed
void fnSmpteGeneratorTimerIsr_2(BOOLEAN bIsRolloverAdded_2)
{

    SMPTE_SKEWING *psrSmpteSkewing_2; // using pointers is faster

	//UINT16 usCurrentMatchValue;
    UINT    uiSmpteMatchValue_2;
    UINT    uiTempTicks_2;

    BOOLEAN bIsNextOutputHigh_2;
    BOOLEAN bIsSmpteTopOfSecond_2;
    
    UINT uiCurrentTicks_2;

    INT iSkewDiff_2;

    static UINT uiIsApply_2;
    
    g_ucEdgeNumber_2++;
    if (g_ucEdgeNumber_2 >= DF_SKEW_TEMP_MAX)
        g_ucEdgeNumber_2 = 0;
    

    bIsSmpteTopOfSecond_2 = false;
    psrSmpteSkewing_2     = &g_srSmpteGenSkewing_2;
	// g_usSmpteMatchValue is constant for every run of this func. It is set by type of time code being generated.
    uiSmpteMatchValue_2   = g_usSmpteMatchValue_2; // global depends on type of smpte. = freq div by num-half-bits.
    bIsNextOutputHigh_2   = g_bSmpteNextOutputStateIsHigh_2;

    // roughly in the middle of a second, update the skewing
    if ((g_ucSmpteFrame_Counter_2 == 12) && (g_ucSmpteCurrentBitNumber_2 == 4) && (!g_bIsHalfBit_2))
        //&& (!(g_srRC600Control.ucOpMode & (SMPTE_COUNT_MASK | SMPTE_INIT_MASK)))) // rc600 not applicable to rc1000 smpte 2
    {
        // kw comment: this to limit calcs to once per second?
      
        if (psrSmpteSkewing_2->m_bSmpteTosCaptured) // This is true when pps generated at system tos, so cannot be used for dropframe.
        {
            psrSmpteSkewing_2->m_bSmpteTosCaptured = false;
            
            if (!g_bDropFrameGenerating_2) // kw v dropframe - see idle thread for jam of df.
            {
                // If we are within three frames of the reference then skewing is allowed.
                // kw comment: this 'if' is for the opposite, when we need to jam sync.
                if ((psrSmpteSkewing_2->m_uiSmpteFrameAtTopOfSecond > 3) && 
                    (psrSmpteSkewing_2->m_uiSmpteFrameAtTopOfSecond < (g_ucSmpteTcgMaxFrameValue_2 - 3)))
                {
                    // what to do about LEDs in RC1000?
         //           LED_YELLOW_NTP_LONG_BLINK_ISR();
         //           LED_GREEN_LONG_BLINK_ISR();

                    // We need to do a jam sync
                    // This is done outside of an interrupt routine.
                    
                    // The idle thread will trigger off this near the half second to do
                    // g_srMsecThread.uiStartTimeCodeGenerator = 1;
                    // and
                    // fnMqxSemaphorePost(&g_srSemaphoreId.m_lwPriorityEventSemaphore,"START_TC");
                    // See IdleThread.c line 212
                    
                    g_srMsecThread.uiStartTimeCodePending = START_TIME_CODE_PENDING_SMPTE_2;
                    g_uiStartTimeCodeData1 = psrSmpteSkewing_2->m_uiSmpteFrameAtTopOfSecond;
                    g_uiStartTimeCodeData2 = psrSmpteSkewing_2->m_uiSmpteBitCountAtTopOfSecond;
                    return;
                }
            }

            // kw comment: don't see why "we are within one bit of being correct" below.
            //
            // We know we are within one bit of being correct.
            // We know that the top of second time comes from the system, therefore
            // we cannot have an error in time of more than one bit.
            //

            // g_ullIsrFtm3_TC_AtTopOfSecond contains the value of FTM3_CH3 at the system's top of second
            // We compare this to the value of the time code generators counter value at top of second.
            // If FTM3 is running at 10 Mhz, 16 bits rolls over in 6.5536 milleseconds.
            // If FTM3 is running at 15 Mhz, 16 bits rolls over in 4.3691 milleseconds.
            // The maximum we will allow time code to skew is 2 milleseconds per second.
            // We know timecode was started within a few microseconds of top of second.
            // As long as time code can skew at a rate greater than the system skewing,
            // time code can never be more than 2 milleseconds from system top of second
            // unless a jam sync occurred. If this happens, we will be more than
            // 2 bits off and time code will also do a jam sync.
            
            // def of g_ullIsrFtm3_TC_MatchAtTopOfSecond is in flextimer.c.
            // It is used because the timer for smpte, FTM3_CH1, is free running, so it is not set back to 0 on match. 
            // Thus we cannot just save a match value once and then repeatedly compare the timer to it.
            // Instead we have to update the match value, repeatedly adding to it to "stay ahead" of the timer. 
            // So we add to g_ullIsrFtm3_TC_MatchAtTopOfSecond.
            // It is set to 0 in flextimer.c and below, for smpte. It's also used for irig.
            
            uiIsApply_2++;
            if (uiIsApply_2 > 1)
            {
                uiIsApply_2 = 0;
            }
            if (!uiIsApply_2)
            {
                // If we get a large difference, wait three seconds before we use it
                // This is a crude filter/hack. We were observing times when there was a difference of 50000 ticks
                // for no apparent reason. Have not found the root cause. This works around the issue.
                // I DON'T LIKE THIS SOLUTION!!!
                if (g_ullIsrFtm3_TC_AtTopOfSecond >= g_ullIsrFtm3_TC_MatchAtTopOfSecond_2)
                {
                    iSkewDiff_2 = (INT)(g_ullIsrFtm3_TC_AtTopOfSecond - g_ullIsrFtm3_TC_MatchAtTopOfSecond_2);
                } else
                {
                    iSkewDiff_2 = (INT)(g_ullIsrFtm3_TC_MatchAtTopOfSecond_2 - g_ullIsrFtm3_TC_AtTopOfSecond);
                }

                // need 2nd incarnation of this?
                
                g_ulIsrFtm3_LastDiff[3] = iSkewDiff_2;

                if (iSkewDiff_2 >= 5000)
                {
                    if ((g_ulIsrFtm3_LastDiff[0] < 5000) && (g_ulIsrFtm3_LastDiff[1] < 5000) && (g_ulIsrFtm3_LastDiff[2] < 5000))
                    {
                        iSkewDiff_2 = 0;
                    }
                }

                g_ulIsrFtm3_LastDiff[0] = g_ulIsrFtm3_LastDiff[1];
                g_ulIsrFtm3_LastDiff[1] = g_ulIsrFtm3_LastDiff[2];
                g_ulIsrFtm3_LastDiff[2] = g_ulIsrFtm3_LastDiff[3];

                if (g_ullIsrFtm3_TC_AtTopOfSecond >= g_ullIsrFtm3_TC_MatchAtTopOfSecond_2)
                {
                    // time code is behind system time, need to
                    // speed up by adding ticks
                    // kw comment: is this really the # of ticks? Isn't it just the dif between counts?
                    psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks = iSkewDiff_2;
                } else
                {
                    // time code is ahead of system time, need to
                    // slow down up by subtracting ticks
                    psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks  = iSkewDiff_2;
                    psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks  = -psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks;
                }
            } else
            {
                // We only skew to the reference every other second
                psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks = 0;
            }

            // Since SMPTE timing does not divide into our clock ticks, we must add in the number of ticks we lost.
            psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks += g_usSmpteModulatorSkewPreload_2; // rhs is always 0, so no change.

            // two edges per bit. we are skewing on every edge
            psrSmpteSkewing_2->m_iNextMcrSmpteSkewPerEdge = (psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks/g_usSmpteTotalNumHalfBits_2) + 1;

            // kw comment: above says limit is 2ms/s
            // limit skewing to 10 msec/second
            if (psrSmpteSkewing_2->m_iNextMcrSmpteSkewPerEdge > (g_uiSmpteMaxSkewPerBit_2/2)) // two edges per bit
            {
                psrSmpteSkewing_2->m_iNextMcrSmpteSkewPerEdge = g_uiSmpteMaxSkewPerBit_2/2;
            }
            
            
            // When dropframe is being generated its TOS never aligns with system TOS
            // so the skewing scheme for df is to NOT use the above calcs that derive from SMPTE TOS,
            // but instead to use the skew that the system used for its TOS.
            
            
			if (g_bDropFrameGenerating_2)
            {
                psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks = 0;
                psrSmpteSkewing_2->m_iNextMcrSmpteSkewPerEdge = 0;
                
                // The skewing scheme for other tc is not workable for dropframe.
                // See the idle thread.
            }    
        } // end if smpte tos captured.
    } // end update skewing in middle of second.

		//If true, we are at the half bit setting up for the start of the bit
		if(g_bIsHalfBit_2)
		{
			g_bIsHalfBit_2 = false;
			g_ucSmpteCurrentBitNumber_2++;

            
            // kw v dropframe - df has to be started at the right ms.
            if (g_bDropFrameGenerating_2 && g_bDF_Sync_Now_2) // if startframe has been calculated. 
            {
                g_bDF_Sync_Now_2 = false; 
                g_ucSmpteFrame_Counter_2 = g_ucSmpteTcgMaxFrameValue_2; // force SetSmpteStart() to return true.
                g_ucSmpteCurrentBitNumber_2 = MAX_SMPTE_BITS; 
            }
            // end kw
                
            
			// If at end of SMPTE frame (Maximum bits) start next frame.
            // This causes skewing #s calculated above ('next') to be put into use.
            // Why wait until new second to do this?
            
			if(g_ucSmpteCurrentBitNumber_2 >= MAX_SMPTE_BITS) // 80 bits per frame
			{

                bIsSmpteTopOfSecond_2 = fnSetSmpteStart_2(); // next func. Work at start of every frame. TOS is when frames max.

                if (bIsSmpteTopOfSecond_2) // Following runs only at SMPTE TOS.
                {
                  
                    // Get the skewing values that were saved in the previous frame
                    psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks = psrSmpteSkewing_2->m_iNextMcrSmpteSkewClockTicks;
                    psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge    = psrSmpteSkewing_2->m_iNextMcrSmpteSkewPerEdge;
                    
                   if (g_bDropFrameGenerating_2)
                   {
                        // A channel can be set to either mode 'input capture' or 'output match'.
                        // If 'input compare' then its value FTMx_CyV cannot be set by code; it is set by the processor. 
                        // If 'output match' then code can put a number into FTMx_CyV,
                        // so the mode must be right before we do this.
                  
                        // Here we are at smpte dropframe tos, so send out a pulse, 
                        // the purpose being to capture the ticks per smpte second.
                        // (Ftm0 channel 6 was previously unused in generators.)
                        // Set match value of Ftm0 channel 6 to the current # of ticks in 1 ms plus the system's current milliseconds
                        // so that in 1 ms a pulse appears. The ftm0 interrupt routine has code added to clear the pulse.
                        
                        uiCurrentTicks_2 = (UINT)FTM0_CNT;
                            // 1 ms of ticks + current ms of ticks. Why not just add reaload?
                        FTM0_C6V    = (UINT)g_uiFTM0_MsecReload + uiCurrentTicks_2; // Set the match value of the timer to now + 1 ms.
                        
                        // Clear the channel status of interrupts.
                        if (FTM0_C6SC & 0x80)
                        {
                            FTM0_C6SC &= ~0x80;
                        }

                        // The pulse is cleared in the FTM0 interrupt routine, channel 6, isrINT_FTM0 in flextimer.c.
                        // Channel 3: clear any pending interrupts, enable interrupts, set output on match
                        FTM0_C6SC = (UINT32)0x5CUL;     // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=1,DMA=0
                        
                        // Figure the current ticks so we can see how many have passed per 30 frames of df.
                        //g_ullDFticks2 = (UINT64)g_ullIsrFTM0_OverflowOffset + (UINT64)uiCurrentTicks;
                        //g_ullDF_MeasuredTicksPerSecond = (UINT64)g_ullDFticks2 - (UINT64)g_ullDFticks1; // used in idlethread.
                        //g_ullDFticks1 = g_ullDFticks2; // Save so we can calc next time.
                         
                        #ifdef _DEBUG_TCG_DROPFRAME
                            LED_YELLOW_NTP_ONE_BLINK_ISR(); // kw testing - visual aid.
                        #endif

                        // Calculate how far away from system tos the smpte tos is. Idle thread prints out.
                        g_ui_mS = g_uiCurrent_mS;
                        if (g_ui_mS <= 500) // smpte tos is after system tos
                        {
                            g_iSmpteTosPrevious = g_ui_mS;
                        }
                        else // smpte tos is before system's.
                        {
                            g_ui_mS = 1000 - g_ui_mS;
                  
                            g_iSmpteTosPrevious = 0 - g_ui_mS;
                        }
                        
                        // Here at drop frame TOS we copy the skew array from the temp that the idle thread filled in
                        // to the working array that is going to be added to the match value.
                        memcpy( g_aucDFskew, g_aucDFskew_temp, sizeof(g_aucDFskew));
  
                    } // end if df
					// end kw v dropframe

                } // end if smpte tos

			} // end bit# > max
			else// if not at end of frame set up for next bit
			{
				g_ucSmpteCurrentBitMask_2 <<= 1;
				if(!g_ucSmpteCurrentBitMask_2)
				{
					g_ucSmpteCurrentBitMask_2 = 0x01;
					g_ucSmpteCurrentByte_2++;
				}
			}
			g_bDatatBit_Smpte_2 = (BOOLEAN)(g_aucSmpteCurrentBuffer_2[g_ucSmpteCurrentByte_2] & g_ucSmpteCurrentBitMask_2);

            // toggle output on match
            if (bIsNextOutputHigh_2) // was set to global at func start
            {
                bIsNextOutputHigh_2 = false;
            } else
            {
                bIsNextOutputHigh_2 = true;
            }
		}
		else // not at half bit
		{
			g_bIsHalfBit_2 = true;
			if(g_bDatatBit_Smpte_2)
			{
				// If we are sending a '1' then toggle the output
				// which is the mode we are already set to
                if (bIsNextOutputHigh_2)
                {
                    bIsNextOutputHigh_2 = false;
                } else
                {
                    bIsNextOutputHigh_2 = true;
                }
			} else
			{
				// We are not sending a '1' so do not toggle the pin on the next match
				// Just generate an interrupt
			}
		}



    // Update the reload for any skewing needed. // kw: when is this done? Every frame? Why? 
	// Ticks and Edge are only calculated at TOS, and this code runs every frame, so dec ClockTicks by PerEdge every frame.
    // ClockTicks is the total skew to apply throughout the frame, and PerEdge is the amount to apply each time through here.
	// uiSmpteMatchValue starts out as the same for every call of this func,
	// so must repeatedly be changed by the amount to skew each edge.
	
    if (psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks) // if any ticks to skew
    {
        if (psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks >= 0) // if positive number of ticks
        {
			// kw comment: don't understand why test has ticks on left and edge on right.
			// PerEdge is ClockTicks/numhalfbits, so will PerEdge will always be less than ClockTicks.
            // Maybe reason is that PerEdge is forced down to a max value if too high.
            // Maybe also to avoid decrementing ClockTicks below 0.
			
            if (psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks >= psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge)
            {
                uiSmpteMatchValue_2                                 += (UINT)psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge;
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks -= psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge;
            } 
			else // The skews-per-edge have been used up, so just use up the remaining clock ticks.
            {
                uiSmpteMatchValue_2                                 += psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks;
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks  = 0;
            }

            if (psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks < 0)
            {
                // !!! cannot happen !!!
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks--;
            }
        } else // the clock ticks are negative
        {
            
            uiTempTicks_2 = (UINT)(-psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks);

            if (uiTempTicks_2 >= psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge)
            {
                uiSmpteMatchValue_2                                 -= (UINT)psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge;
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks += psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewPerEdge;  // remember, we are adding to a negative number
            } else
            {
                uiSmpteMatchValue_2                                 -= uiTempTicks_2;
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks  = 0;
            }

            if (psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks > 0)
            {
                // !!! cannot happen !!!
                psrSmpteSkewing_2->m_iCurrentMcrSmpteSkewClockTicks++;
            }
        }
    } // end if any ticks to skew
    

    // kw v dropframe - the skew for df is in the elements of the array. 
    // Each element is 1 at most. Others are 0. Array was copied above at tos.
    if (g_bDropFrameGenerating_2)
    {   
        uiSmpteMatchValue_2 += g_aucDFskew_2[g_ucEdgeNumber_2]; 
    }
    

    // Condition the output depending on what the next state should be.
    if (bIsNextOutputHigh_2 != g_bSmpteNextOutputStateIsHigh_2)
    {

       // make sure the timer is stopped
       //FTM3_MODE |= (UINT32)0x05UL;                       // disable write protection, enable all register access
       //FTM3_SC    = (UINT32)((FTM3_SC & (UINT32)0xE7UL)); // no clock selected, disables FTM

       g_bSmpteNextOutputStateIsHigh_2 = bIsNextOutputHigh_2;

       // guts
       if (bIsNextOutputHigh_2)
       {
           // Channel 2, enable interrupts, set the output on match
           
           FTM3_C2SC = (UINT32)0x5CUL; // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=1,res=0,DMA=0
           
       } else
       {
           // Channel 2, enable interrupts, clear the output on match
           
           FTM3_C2SC = (UINT32)0x58UL; // CHF=0,CHIE=1,MSB=0,MSA=1,ELSB=1,ELSA=0,res=0,DMA=0
       }
    }

	// remember, FTM3_CH2 is free running, we do not reload on a match

    // If the next edge is time code top of second, save the counter value.
    if (bIsSmpteTopOfSecond_2)
    {
        if (bIsRolloverAdded_2)
        {
            if ((UINT)FTM3_C2V > (UINT)0x8000)
            {
                g_ullIsrFtm3_TC_MatchAtTopOfSecond_2 = (g_ullIsrFTM3_OverflowOffset - (UINT64)0x10000) + (UINT64)FTM3_C2V + (UINT64)uiSmpteMatchValue_2; 
            } else
            {
                g_ullIsrFtm3_TC_MatchAtTopOfSecond_2 = g_ullIsrFTM3_OverflowOffset + (UINT64)FTM3_C2V + (UINT64)uiSmpteMatchValue_2; 
            }
        } else
        {
            g_ullIsrFtm3_TC_MatchAtTopOfSecond_2 = g_ullIsrFTM3_OverflowOffset + (UINT64)FTM3_C2V + (UINT64)uiSmpteMatchValue_2; 
        }
    }

    // as long as FTM3_C2SC is set to the correct mode, we can update the match value
    // without stopping the timer before setting it

    FTM3_C2V += uiSmpteMatchValue_2; // guts

    // How to avoid conflict with ch 1 in following. May need g_ullIsrFTM3_OverflowOffset_2
    // is there a pending rollover we need to add?
    if (FTM3_CNT < 0x8000)
    {
        if (FTM3_SC & 0x80)
        {
            // there was an overflow, clear the overflow flag
            // and add the overflow offset
            FTM3_SC                       &= ~0x80;
            g_ullIsrFTM3_OverflowOffset += (UINT64)0x00010000;
        }
    }

} // end SmpteGeneratorTimerIsr_2




//*****************************************************************************
//      Name:           fnSetSmpteStart()
//
//      Description:	The next SMPTE timer interrupt is the start of a SMPTE frame.
//                      Returns true at smpte TOS.
//                      Fill in g_aucSmpteCurrentBuffer with RC600 count or next-second-buffer,
//                      set bits for dropframe, polarity, etc.
//
//		Notes:          Runs at the start of every smpte frame
//                      and when tc config changes, at startup, on jam sync.
//
//		MP safe:		Unknown
//
#pragma optimize=speed
BOOLEAN fnSetSmpteStart(void)
{
    BOOLEAN bIsTopOfSecond;
	UINT8  ucPolarity;// bit 27 the polarity bit is unassignned in CA encoding
	UINT8  ucSmpteByte;
	UINT8  ucBitMask;
	UINT8 ucTempValue;
    UINT8 ucZeroMask;


    bIsTopOfSecond = false;
	ucPolarity     = 0;

    // RC600 output - RC600 SMTPE - RC600 count
    // The count mask covers any OP_MODE_UP_x, OP_MODE_DOWN_x and OP_MODE_COUNT_2.
    // The init mask covers up-ready, set-start, set-count, preempt, down-count, x_init, x_ext, x_stop and ledtest.
	if (g_srRC600Control.ucOpMode & (SMPTE_COUNT_MASK | SMPTE_INIT_MASK)){

        // Put info into the frame to tell things to the receving device.
        // This creates a frame that is NOT standard SMPTE, so the receiving device must be set accordingly.
        // The BIT on the right of the lines below is the info to convey.
        // 

        g_ucSmpteFrame_Reset++;
        g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP1] |= SMPTE_RC600_CONTROL_BIT; //RC600 control in progress
        
        if (g_ucSmpteFrame_Reset > g_ucSmpteTcgMaxFrameValue)
            g_ucSmpteFrame_Reset   = 0;
        
        if (g_srRC600Control.bBlankDisplay)
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP8] |= SMPTE_RC600_BLANKS_BIT;
        else
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP8] &= ~SMPTE_RC600_BLANKS_BIT;
        
        if (g_srRC600Control.bBlankZeros)
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP8] |= SMPTE_RC600_ZEROS_BIT;
        else
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP8] &= ~SMPTE_RC600_ZEROS_BIT;

        if (g_srRC600Control.bDashesDisplay){
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP6] |= SMPTE_RC600_DASHES_BIT;
            ucZeroMask = 0x00;
        } else {
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_BINARY_GROUP6] &= ~SMPTE_RC600_DASHES_BIT;
            ucZeroMask = 0xFF;
        } 
        
        if (g_srRC600Control.bKillFrames){
            g_ucSmpteFrame_Counter = 0;
            g_ucSmpteFrame_ones    = 0;
            g_ucSmpteFrame_tens    = 0;
            
            g_srRC600Control.bKillFrames = false;
        }

        if (g_srRC600Control.bSmpteSetCount){
       
            if (g_srRC600Control.bIsTimerRunning || g_srRC600Control.bCount2Time ||
                (g_srRC600Control.ucOpMode & SMPTE_INIT_MASK)){
                g_ucSmpteFrame_Counter = 0;
                g_ucSmpteFrame_ones    = 0;
                g_ucSmpteFrame_tens    = 0;
            }
          
            g_srRC600Control.bSmpteSetCount = false;
            
            // Don't understand why this encoding is being done.

            //Sec
            ucTempValue = (bin_to_BCD(g_srRC600Control.ucSeconds)) & ucZeroMask;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_SECOND_ONES] &= 0xF0; 
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_SECOND_ONES] |= (ucTempValue & 0x0F);       //units
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_SECOND_TENS] &= 0xF8;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_SECOND_TENS] |= ((ucTempValue >> 4) & 0x07);  //tens

            //Min
            ucTempValue = (bin_to_BCD(g_srRC600Control.ucMinutes)) & ucZeroMask;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_MINUTE_ONES] &= 0xF0;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_MINUTE_ONES] |= (ucTempValue & 0x0F);       //units
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_MINUTE_TENS] &= 0xF8;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_MINUTE_TENS] |= ((ucTempValue >> 4) & 0x07);  //tens

            //Hour
            ucTempValue = (bin_to_BCD(g_srRC600Control.ucHours % 24)) & ucZeroMask;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_HOUR_ONES] &= 0xF0; 
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_HOUR_ONES] |= (ucTempValue & 0x0F);         //units
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_HOUR_TENS] &= 0xFC;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_HOUR_TENS] |= ((ucTempValue >> 4) & 0x03);    //tens

            //Days (needed for Hours > 24)
            ucTempValue = (bin_to_BCD(g_srRC600Control.ucHours / 24)) & ucZeroMask;
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_LEITCH_DATE_ONES] &= 0x0F; 
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_LEITCH_DATE_ONES]  |= ((ucTempValue<<4) & 0xF0);
            g_aucSmpteCurrentBuffer[SMPTE_BYTE_LEITCH_DATE_TENS] &= 0xCF; 
                      
            g_ucPolarity = 0;
             
            for(ucSmpteByte = 0;  ucSmpteByte < 8; ucSmpteByte++)// count '1's
            {
                for(ucBitMask = 0x01; ucBitMask; ucBitMask <<= 1)
                {
                    if(g_aucSmpteCurrentBuffer[ucSmpteByte] & ucBitMask)
                    {
                        g_ucPolarity++;//count the '1's
                    }
                }
            }

        } // end if bSmpteSetCount
        else if (g_srRC600Control.bIsTimerRunning || g_srRC600Control.bCount2Time){

            g_ucSmpteFrame_Counter++;

            if (g_ucSmpteFrame_Counter > g_ucSmpteTcgMaxFrameValue){
                g_ucSmpteFrame_Counter = 0;
            }
            
            if (g_srRC600Control.bIsCountUp){

                g_ucSmpteFrame_ones    = g_ucSmpteFrame_Counter % 10;
                g_ucSmpteFrame_tens    = g_ucSmpteFrame_Counter / 10;                       
                
            } else {
                
                g_ucSmpteFrame_ones    = (g_ucSmpteTcgMaxFrameValue - g_ucSmpteFrame_Counter) % 10;
                g_ucSmpteFrame_tens    = (g_ucSmpteTcgMaxFrameValue - g_ucSmpteFrame_Counter) / 10;  
            }
        }
        
    } else // not rc600 count or init
    { 
        if (g_srRC600Control.bSmpteReset){
            
            g_ucSmpteFrame_Counter = g_ucSmpteFrame_Reset;
            g_ucSmpteFrame_ones = g_ucSmpteFrame_Counter % 10;
            g_ucSmpteFrame_tens = g_ucSmpteFrame_Counter / 10;
            g_srRC600Control.bSmpteReset = false;
            g_aucSmpteNextSecondBuffer[SMPTE_BYTE_BINARY_GROUP1] &= ~SMPTE_RC600_CONTROL_BIT; // exit RC600 control mode
            memcpy(g_aucSmpteCurrentBuffer, g_aucSmpteNextSecondBuffer, sizeof(g_aucSmpteCurrentBuffer));
        }

        if (g_ucSmpteFrame_Counter >= g_ucSmpteTcgMaxFrameValue) // Time to start frame 0.
        {
            // Update the time we use
             
            memcpy(g_aucSmpteCurrentBuffer, g_aucSmpteNextSecondBuffer, sizeof(g_aucSmpteCurrentBuffer));
            
            bIsTopOfSecond = true;
            g_ucSmpteFrame_Reset   = 0;
            g_ucSmpteFrame_Counter = 0; //g_bDropFrameGenerating
            g_ucSmpteFrame_ones    = 0;
            g_ucSmpteFrame_tens    = 0;
            
            // kw v dropframe - when systime sees seconds == 0 and minutes not a multiple of 10
            // then we drop frames 0 and 1 to start with frame == 2.
            // This is the method whereby dropframe stays accurate thru the day.
            if (g_bDropFrameGenerating && g_bDropFrameTopOfMinute)
            {
                g_ucSmpteFrame_Counter = 2;
                g_ucSmpteFrame_ones    = 2; // drop frames 0 and 1

                g_bDropFrameTopOfMinute = false;
                g_bDropFrameFramesDropped = true;
            }
            
            g_ucPolarity = 0;
            
            for(ucSmpteByte = 0;  ucSmpteByte < 8; ucSmpteByte++)// count '1's
            {
                for(ucBitMask = 0x01; ucBitMask; ucBitMask <<= 1)
                {
                    if(g_aucSmpteCurrentBuffer[ucSmpteByte] & ucBitMask)
                    {
                        g_ucPolarity++;//count the '1's
                    }
                }
            }                

        } else // frame not over max
        {
            g_ucSmpteFrame_Counter++;
            g_ucSmpteFrame_Reset++;
                
            g_ucSmpteFrame_ones++;
            if (g_ucSmpteFrame_ones == 10)
            {
                g_ucSmpteFrame_ones = 0;
                g_ucSmpteFrame_tens++;
            }
            
        }
 
    }
	//set non-user defined bits (bits 0-3)

	//clear Frames units bits
	g_aucSmpteCurrentBuffer[SMPTE_BYTE_FRAME_ONES] &=  0xF0;

	//write Frames units to buffer
	g_aucSmpteCurrentBuffer[SMPTE_BYTE_FRAME_ONES] |= (g_ucSmpteFrame_ones & 0x0F);	//	bin_to_BCD	((	))

	//set byte 2 Frame Tens (bits 8-9) (bits 10-11 are the drop frame flag and color flag)
	//clear Frames tens bits
	g_aucSmpteCurrentBuffer[SMPTE_BYTE_FRAME_TENS] &=  0xF0;

	//write Frames tens to buffer
	g_aucSmpteCurrentBuffer[SMPTE_BYTE_FRAME_TENS] |= g_ucSmpteFrame_tens; //bin_to_BCD((  ) ) >>4
    
    // kw v dropframe - set the dropframe bit
    if (g_bDropFrameGenerating)
    {
        g_aucSmpteCurrentBuffer[SMPTE_BYTE_FRAME_TENS] |= 0x04;
    }
    
	//set polarity bit to '0'
	g_aucSmpteCurrentBuffer[3] &= 0xF7;

	// Bi-phase correction bit
	ucPolarity += g_aucSmpteCountOnesBCDLookUp[g_ucSmpteFrame_ones];
	ucPolarity += g_aucSmpteCountOnesBCDLookUp[g_ucSmpteFrame_tens];

	ucPolarity += g_ucPolarity;

	if(!(ucPolarity & 0x01))// == 0
	{
		// Bi-phase correction bit
		switch(g_srSmpteCurrentConfig.m_ucSmpteFrameRate)
		{
            // For EBU the polarity (a.k.a. parity) bit is #59, unlike others.
			case SMPTE_FRAME_RATE_25:
				g_aucSmpteCurrentBuffer[7] |= 0x08;// set polarity bit
				break;

			case SMPTE_FRAME_RATE_24:
			case SMPTE_FRAME_RATE_30:
            case SMPTE_FRAME_RATE_DROPFRAME:
              
				g_aucSmpteCurrentBuffer[3] |= 0x08;// set polarity bit
				break;
		}

	}

	g_ucSmpteCurrentBitMask   = 0x01;
	g_ucSmpteCurrentByte      = 0;
	g_ucSmpteCurrentBitNumber = 0;
    g_ucEdgeNumber = 0;

    return bIsTopOfSecond;

} // end fnSetSmpteStart





#pragma optimize=speed
BOOLEAN fnSetSmpteStart_2(void)
{
    BOOLEAN bIsTopOfSecond;
	UINT8  ucPolarity;// bit 27 the polarity bit is unassignned in CA encoding
	UINT8  ucSmpteByte;
	UINT8  ucBitMask;
	//UINT8 ucTempValue;
    //UINT8 ucZeroMask;


    bIsTopOfSecond = false;
	ucPolarity     = 0;

// deleted rc600 code
    
        if (g_ucSmpteFrame_Counter_2 >= g_ucSmpteTcgMaxFrameValue_2) // Time to start frame 0.
        {
            
            // Update the time we use
             
            memcpy(g_aucSmpteCurrentBuffer_2, g_aucSmpteNextSecondBuffer_2, sizeof(g_aucSmpteCurrentBuffer_2));
            
            bIsTopOfSecond = true;
            g_ucSmpteFrame_Reset_2   = 0;
            g_ucSmpteFrame_Counter_2 = 0;
            g_ucSmpteFrame_ones_2    = 0;
            g_ucSmpteFrame_tens_2    = 0;
            

            // kw v dropframe - when systime sees seconds == 0 and minutes not a multiple of 10
            // then we drop frames 0 and 1 to start with frame == 2.
            // This is the method whereby dropframe stays accurate thru the day.
            if (g_bDropFrameGenerating_2 && g_bDropFrameTopOfMinute_2)
            
            {
                g_ucSmpteFrame_Counter_2 = 2;
                g_ucSmpteFrame_ones_2    = 2; // drop frames 0 and 1

                g_bDropFrameTopOfMinute_2 = false;
                g_bDropFrameFramesDropped_2 = true;
            }
            
            g_ucPolarity_2 = 0;
            
            for(ucSmpteByte = 0;  ucSmpteByte < 8; ucSmpteByte++)// count '1's
            {
                for(ucBitMask = 0x01; ucBitMask; ucBitMask <<= 1)
                {
                    if(g_aucSmpteCurrentBuffer_2[ucSmpteByte] & ucBitMask)
                    {
                        g_ucPolarity_2++;//count the '1's
                    }
                }
            }
        } else // frame not over max
        {
            g_ucSmpteFrame_Counter_2++;
            g_ucSmpteFrame_Reset_2++;
                
            g_ucSmpteFrame_ones_2++;
            if (g_ucSmpteFrame_ones_2 == 10)
            {
                g_ucSmpteFrame_ones_2 = 0;
                g_ucSmpteFrame_tens_2++;
            }
        }
 

	//set non-user defined bits (bits 0-3)

	//clear Frames units bits
	g_aucSmpteCurrentBuffer_2[SMPTE_BYTE_FRAME_ONES] &=  0xF0;

	//write Frames units to buffer
	g_aucSmpteCurrentBuffer_2[SMPTE_BYTE_FRAME_ONES] |= (g_ucSmpteFrame_ones_2 & 0x0F);	//	bin_to_BCD	((	))

	//set byte 2 Frame Tens (bits 8-9) (bits 10-11 are the drop frame flag and color flag)
	//clear Frames tens bits
	g_aucSmpteCurrentBuffer_2[SMPTE_BYTE_FRAME_TENS] &=  0xF0;

	//write Frames tens to buffer
	g_aucSmpteCurrentBuffer_2[SMPTE_BYTE_FRAME_TENS] |= g_ucSmpteFrame_tens_2; //bin_to_BCD((  ) ) >>4
    
    // kw v dropframe - set the dropframe bit
    if (g_bDropFrameGenerating_2)
    {
        g_aucSmpteCurrentBuffer_2[SMPTE_BYTE_FRAME_TENS] |= 0x04;
    }
    
	//set polarity bit to '0'
	g_aucSmpteCurrentBuffer_2[3] &= 0xF7;

	// Bi-phase correction bit
	ucPolarity += g_aucSmpteCountOnesBCDLookUp[g_ucSmpteFrame_ones_2];
	ucPolarity += g_aucSmpteCountOnesBCDLookUp[g_ucSmpteFrame_tens_2];

	ucPolarity += g_ucPolarity_2;

	if(!(ucPolarity & 0x01))// == 0
	{
		// Bi-phase correction bit
		switch(g_srSmpteCurrentConfig_2.m_ucSmpteFrameRate)
		{
            // For EBU the polarity (a.k.a. parity) bit is #59, unlike others.
			case SMPTE_FRAME_RATE_25:
				g_aucSmpteCurrentBuffer_2[7] |= 0x08;// set polarity bit
				break;

			case SMPTE_FRAME_RATE_24:
			case SMPTE_FRAME_RATE_30:
            case SMPTE_FRAME_RATE_DROPFRAME:
              
				g_aucSmpteCurrentBuffer_2[3] |= 0x08;// set polarity bit
				break;
		}

	}

	g_ucSmpteCurrentBitMask_2   = 0x01;
	g_ucSmpteCurrentByte_2      = 0;
	g_ucSmpteCurrentBitNumber_2 = 0;
    g_ucEdgeNumber_2 = 0;

    return bIsTopOfSecond;
    
} // end SetSmpteStart_2



// kw rc1000 - copied from gps
BOOLEAN fnLockSmpte(char *pName)
{
    #ifdef _DEBUG_USE_SEMAPHORE
    BOOLEAN         bSemStatus;
    #endif
    
    #ifdef _DEBUG_USE_MUTEX
    BOOLEAN         bMutexStatus;
    #endif


//  #ifdef _DEBUG_ERROR
//      debugPrintf("{{%s\n", pName);
//  #endif
    #ifdef _DEBUG_USE_SEMAPHORE
    bSemStatus = fnMqxSemaphoreWait(&g_srSemaphoreId.m_lwSmpte_Semaphore;, TICK_1_SEC, pName, SEM_TIMEOUT_IS_NOT_OKAY);
    if (bSemStatus) // if error
    {} // for compiler warning
    #endif
    
    #ifdef _DEBUG_USE_MUTEX
    bMutexStatus = fnMqxMutexLock(&g_srMutexId.m_MutexSmpte, pName);   
    if (bMutexStatus) // if error
    {} // for compiler warning
    #endif

    #ifdef _DEBUG_USE_SEMAPHORE
    return bSemStatus;
    #else
    return bMutexStatus;  
    #endif    
}



// kw rc1000 - copied from gps
BOOLEAN fnUnlockSmpte(char *pName)
{
    #ifdef _DEBUG_USE_SEMAPHORE
    BOOLEAN         bSemStatus;
    #endif
    
    #ifdef _DEBUG_USE_MUTEX
    BOOLEAN         bMutexStatus;
    #endif

//  #ifdef _DEBUG_ERROR
//      debugPrintf("%s}}\n", pName);
//  #endif
    #ifdef _DEBUG_USE_SEMAPHORE
    bSemStatus = fnMqxSemaphorePost(&g_srSemaphoreId.m_lwSmpte_Semaphore, pName);
    if (bSemStatus)
    {} // for compiler warning
    #endif
    
    #ifdef _DEBUG_USE_MUTEX
    bMutexStatus = fnMqxMutexUnlock(&g_srMutexId.m_MutexSmpte, pName);    
    if (bMutexStatus)
    {} // for compiler warning    
    #endif
    
    #ifdef _DEBUG_USE_SEMAPHORE
    return bSemStatus; 
    #else
    return bMutexStatus;    
    #endif
}
