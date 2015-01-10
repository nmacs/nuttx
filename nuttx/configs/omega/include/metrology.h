/************************************************************************************
 * configs/omega/include/metrology.h
 ************************************************************************************/

#ifndef __CONFIGS_OMEGA_INCLUDE_METROLOGY_H
#define __CONFIGS_OMEGA_INCLUDE_METROLOGY_H

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#define DSP_READING_SIZE 512

#define DSP_SAMPLES_PES_SECOND (4000)
//=========================================================

//=========================================================

//=========================================================
typedef	enum
{
    STATE_CTRL_Os		= 0,
    FEATURE_CTRL0_Os,
    FEATURE_CTRL1_Os,
    METER_TYPE_Os,
    M_Os,
    N_MAX_Os,
    PULSE0_CTRL_Os,
    PULSE1_CTRL_Os,
    PULSE2_CTRL_Os,
    P_K_T_Os,
    Q_K_T_Os,
    I_K_T_Os,
    CREEP_THR_P_Os,
    CREEP_THR_Q_Os,
    CREEP_THR_I_Os,
    POWER_OFFSET_CTRL_Os,
    POWER_OFFSET_P_Os,
    POWER_OFFSET_Q_Os,
    SWELL_THR_VA_Os,
    SWELL_THR_VB_Os,
    SWELL_THR_VC_Os,
    SAG_THR_VA_Os,
    SAG_THR_VB_Os,
    SAG_THR_VC_Os,
    K_IA_Os,
    K_VA_Os,
    K_IB_Os,
    K_VB_Os,
    K_IC_Os,
    K_VC_Os,
    RESERVED1_Os,
    CAL_M_IA_Os		= 31,
    CAL_M_VA_Os,
    CAL_M_IB_Os,
    CAL_M_VB_Os,
    CAL_M_IC_Os,
    CAL_M_VC_Os,
    RESERVED2_Os,
    CAL_PH_IA_Os,
    CAL_PH_VA_Os,
    CAL_PH_IB_Os,
    CAL_PH_VB_Os,
    CAL_PH_IC_Os,
    CAL_PH_VC_Os,
    RESERVED3_Os,
    RESERVED4_Os,
    RESERVED5_Os,
    RESERVED6_Os,
    RESERVED7_Os,
    RESERVED8_Os,
    RESERVED9_Os,
    ATsense_CTRL_20_23_Os,
    ATsense_CTRL_24_27_Os,
    ATsense_CTRL_28_2B_Os,
    RESERVED10_Os_Os

} DSP_CTRL_OS_TYPE;
//===================================================================
//---Primary 32-bit Input Control Register Array Size Definitions----
//-------------------------------------------------------------------
typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	ST_CTRL		:	4;	//status control
        uint32_t			:	28;
    } BIT;
} STATE_CTRL_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	ACC_BUF_EN		: 1;	//ACC BUFF ENABLE
        uint32_t			: 3;
        uint32_t	SYNCH			: 2;
        uint32_t			: 2;
        uint32_t	PHASE_A_EN		: 1;	//ENABLE
        uint32_t	PHASE_B_EN		: 1;	//ENABLE
        uint32_t	PHASE_C_EN		: 1;	//ENABLE
        uint32_t			: 1;
        uint32_t	MAX_INT_SELECT	: 1;
        uint32_t			: 19;
    } BIT;
} FEATURE_CTRL0_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	CREEP_I_EN		: 1;	//ENABLE
        uint32_t	CREEP_Q_EN		: 1;	//ENABLE
        uint32_t	CREEP_P_EN		: 1;	//ENABLE
        uint32_t			: 5;
        uint32_t	HARMONIC_m_REQ	: 6;
        uint32_t			: 1;
        uint32_t	HARMONIC_EN		: 1;
        uint32_t			: 3;
        uint32_t	I_MAX_RESET		: 1;
        uint32_t			: 3;
        uint32_t	V_MAX_RESET		: 1;
        uint32_t			: 8;
    } BIT;
} FEATURE_CTRL1_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	SENSOR_TYPE_I_A		: 2;	//
        uint32_t	SENSOR_TYPE_V_A		: 2;	//
        uint32_t	SENSOR_TYPE_I_B		: 2;	//
        uint32_t	SENSOR_TYPE_V_B		: 2;	//
        uint32_t	SENSOR_TYPE_I_C		: 2;	//
        uint32_t	SENSOR_TYPE_V_C		: 2;	//
        uint32_t			: 20;
        //uint32_t	SERVICE_TYPE		: 4;
        //uint32_t			: 1;
        //uint32_t	MISSING_PHASE		: 2;
        //uint32_t	TRANSFORM_EN		: 1;
    } BIT;
} METER_TYPE_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	MV		: 12;	//
        uint32_t			: 20;
    } BIT;
} M_TYPE_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	MAX		: 24;	//
        uint32_t			: 8;
    } BIT;
} N_MAX_TYPE_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	PCx_WIDTH			: 16; //
        uint32_t	PCx_TYPE			: 8;	//
        uint32_t	PCx_POLARITY		: 1;	//
        uint32_t	PCx_OVERRIDE		: 1;	//
        uint32_t	PCx_ACC_HOLD		: 1;	//
        uint32_t			: 1;
        uint32_t	PCx_DETENT			: 2;	//
        uint32_t			: 1;
        uint32_t	PCx_ENABLE			: 1;
    } BIT;
} PULSE_CTRL_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        //uint32_t	PO_VOLTAGE			: 16; //
        uint32_t			: 27;
        uint32_t	REACTIVE_PO_EN		: 1;	//
        uint32_t			: 3;
        uint32_t	ACTIVE_PO_EN		: 1;	//
    } BIT;
} POWER_OS_CTRL_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	CAPTURE_TYPE		: 2; //
        uint32_t			: 2;
        uint32_t	CAPTURE_SOURCE		: 3;	//
        uint32_t			: 1;	//
        uint32_t	CH_SEL_IA			: 1;	//
        uint32_t	CH_SEL_VA			: 1;	//
        uint32_t	CH_SEL_IB			: 1;	//
        uint32_t	CH_SEL_VB			: 1;	//
        uint32_t	CH_SEL_IC			: 1;	//
        uint32_t	CH_SEL_VC			: 1;	//
        uint32_t			: 17;
        uint32_t	CAPTURE_EN			: 1;	//
    } BIT;
} CAPTURE_CTRL_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	CP_BUF_SIZE		: 24; //
        uint32_t			: 8;
    } BIT;
} CAPTURE_BUF_SIZE_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	ACC00_EN		: 1; //
        uint32_t	ACC01_EN		: 1; //
        uint32_t	ACC02_EN		: 1; //
        uint32_t	ACC03_EN		: 1; //
        uint32_t	ACC04_EN		: 1; //
        uint32_t	ACC05_EN		: 1; //
        uint32_t	ACC06_EN		: 1; //
        uint32_t	ACC07_EN		: 1; //
        uint32_t	ACC08_EN		: 1; //
        uint32_t	ACC09_EN		: 1; //
        uint32_t	ACC10_EN		: 1; //
        uint32_t	ACC11_EN		: 1; //
        uint32_t	ACC12_EN		: 1; //
        uint32_t	ACC13_EN		: 1; //
        uint32_t	ACC14_EN		: 1; //
        uint32_t	ACC15_EN		: 1; //
        uint32_t	ACC16_EN		: 1; //
        uint32_t	ACC17_EN		: 1; //
        uint32_t	ACC18_EN		: 1; //
        uint32_t	ACC19_EN		: 1; //
        uint32_t	ACC20_EN		: 1; //
        uint32_t	ACC21_EN		: 1; //
        uint32_t	ACC22_EN		: 1; //
        uint32_t	ACC23_EN		: 1; //
        uint32_t	ACC24_EN		: 1; //
        uint32_t	ACC25_EN		: 1; //
        uint32_t	ACC26_EN		: 1; //
        uint32_t	ACC27_EN		: 1; //
        uint32_t	ACC28_EN		: 1; //
        uint32_t	ACC29_EN		: 1; //
        uint32_t	ACC30_EN		: 1; //
        uint32_t	ACC31_EN		: 1; //
    } BIT;
} ACC_BUFF_CTRL0_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	ACC32_EN		: 1; //
        uint32_t	ACC33_EN		: 1; //
        uint32_t	ACC34_EN		: 1; //
        uint32_t	ACC35_EN		: 1; //
        uint32_t	ACC36_EN		: 1; //
        uint32_t	ACC37_EN		: 1; //
        uint32_t	ACC38_EN		: 1; //
        uint32_t	ACC39_EN		: 1; //
        uint32_t	ACC40_EN		: 1; //
        uint32_t	ACC41_EN		: 1; //
        uint32_t	ACC42_EN		: 1; //
        uint32_t	ACC43_EN		: 1; //
        uint32_t	ACC44_EN		: 1; //
        uint32_t	ACC45_EN		: 1; //
        uint32_t	ACC46_EN		: 1; //
        uint32_t	ACC47_EN		: 1; //
        uint32_t	ACC48_EN		: 1; //
        uint32_t	ACC49_EN		: 1; //
        uint32_t	ACC50_EN		: 1; //
        uint32_t	ACC51_EN		: 1; //
        uint32_t	ACC52_EN		: 1; //
        uint32_t			: 11; //
    } BIT;
} ACC_BUFF_CTRL1_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	I0_ON			: 1; //
        uint32_t	TEMPMEAS		: 1; //
        uint32_t			: 2; //
        uint32_t	I0_GAIN			: 2; //
        uint32_t			: 2; //
        uint32_t	I1_ON			: 1; //
        uint32_t			: 3; //
        uint32_t	I1_GAIN			: 2; //
        uint32_t			: 2; //
        uint32_t	V1_ON			: 1; //
        uint32_t			: 7; //
        uint32_t	I2_ON			: 1; //
        uint32_t			: 3; //
        uint32_t	I2_GAIN			: 2; //
        uint32_t			: 2; //
    } BIT;
} ATsens_CTRL_20_23_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	V2_ON			: 1; //
        uint32_t			: 7; //
        uint32_t	I3_ON			: 1; //
        uint32_t			: 3; //
        uint32_t	I3_GAIN			: 2; //
        uint32_t			: 2; //
        uint32_t	V3_ON			: 1; //
        uint32_t			: 7; //
        uint32_t	ONBIAS			: 1; //
        uint32_t	ONREF			: 1; //
        uint32_t	ONLDO			: 1; //
        uint32_t			: 5; //
    } BIT;
} ATsens_CTRL_24_27_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	OSR				: 2; //
        uint32_t			: 2; //
        uint32_t	MSB_MODE		: 1; //
        uint32_t			: 27; //
    } BIT;
} ATsens_CTRL_28_2B_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	APP_READ		: 1; //
        uint32_t			: 31; //
    } BIT;
} APP_READ_CTRL_TYPE;

typedef	struct
{
    STATE_CTRL_TYPE			STATE_CTRL;				//uQ32.0
    FEATURE_CTRL0_TYPE		FEATURE_CTRL0;
    FEATURE_CTRL1_TYPE		FEATURE_CTRL1;
    METER_TYPE_TYPE			METER_TYPE;

    M_TYPE_TYPE				M;
    N_MAX_TYPE_TYPE			N_MAX;
    PULSE_CTRL_TYPE			PULSE0_CTRL;
    PULSE_CTRL_TYPE			PULSE1_CTRL;

    PULSE_CTRL_TYPE			PULSE2_CTRL;
    uint32_t	P_K_T;				//uQ8.24	Wh/pulse
    uint32_t	Q_K_T;				//uQ8.24	Wh/pulse
    uint32_t	I_K_T;				//uQ8.24	Wh/pulse

    uint32_t	CREEP_THR_P;
    uint32_t	CREEP_THR_Q;
    uint32_t	CREEP_THR_I;
    POWER_OS_CTRL_TYPE		POWER_OFFSET_CTRL;

    int32_t	POWER_OFFSET_P;		//sQ1.30
    int32_t	POWER_OFFSET_Q;		//sQ1.30
    uint32_t	SWELL_THR_VA;
    uint32_t	SWELL_THR_VB;

    uint32_t	SWELL_THR_VC;
    uint32_t	SAG_THR_VA;
    uint32_t	SAG_THR_VB;
    uint32_t	SAG_THR_VC;

    uint32_t	K_IA;				//uQ22.10
    uint32_t	K_VA;				//uQ22.10
    uint32_t	K_IB;
    uint32_t	K_VB;

    uint32_t	K_IC;
    uint32_t	K_VC;
    uint32_t	K_IN;
    int32_t	CAL_M_IA;			//sQ2.29

    int32_t	CAL_M_VA;
    int32_t	CAL_M_IB;
    int32_t	CAL_M_VB;
    int32_t	CAL_M_IC;

    int32_t	CAL_M_VC;
    int32_t	CAL_M_IN;
    int32_t	CAL_PH_IA;			//sQ0.31
    int32_t	CAL_PH_VA;

    int32_t	CAL_PH_IB;
    int32_t	CAL_PH_VB;
    int32_t	CAL_PH_IC;
    int32_t	CAL_PH_VC;

    int32_t	RESERVED3;
    uint32_t	RESERVED4;		//CAPT_CTRL
    uint32_t	RESERVED5;              //CAPT_BUFF_SIZE
    uint32_t	RESERVED6;              //CAPT_ADDR

    uint32_t	RESERVED7;
    uint32_t	RESERVED8;
    uint32_t	RESERVED9;
    ATsens_CTRL_20_23_TYPE	ATsense_CTRL_20_23;

    ATsens_CTRL_24_27_TYPE	ATsense_CTRL_24_27;
    ATsens_CTRL_28_2B_TYPE	ATsense_CTRL_28_2B;
    uint32_t	RESERVED10;
} DSP_CTRL_TYPE;

#define DSP_CONTROL_SIZE (sizeof(DSP_CTRL_TYPE)/sizeof(uint32_t))

typedef struct
{
    uint32_t I_A_R;
    uint32_t V_A_R;
    uint32_t I_B_R;
    uint32_t V_B_R;
    uint32_t I_C_R;
    uint32_t V_C_R;
    uint32_t I_A_I;
    uint32_t V_A_I;
    uint32_t I_B_I;
    uint32_t V_B_I;
    uint32_t I_C_I;
    uint32_t V_C_I;
    uint32_t	RESERVED0;
    uint32_t	RESERVED1;
    uint32_t	RESERVED2;
} DSP_HAR_TYPE;

#define DSP_HAR_SIZE (sizeof(DSP_HAR_TYPE)/sizeof(uint32_t))

//---------------------------------------------------------
#define	DSP_CTRL_ST_CTRL_Reset	0
#define	DSP_CTRL_ST_CTRL_Init	1
#define	DSP_CTRL_ST_CTRL_Run	2
#define	DSP_CTRL_ST_CTRL_Hold	3

typedef	enum
{
    IsReset	= 0,
    IsInit	= 1,
    IsRun	= 2,
    IsHold	= 3,
} DSP_CTRL_ST_CTRL_TYPE;


#define	DSP_CTRL_STATE_CTRL_ST_CTRL_Pos			0
#define	DSP_CTRL_STATE_CTRL_ST_CTRL_Mask		(0xF<<DSP_CTRL_STATE_CTRL_ST_CTRL_Pos)

#define	DSP_CTRL_METER_TYPE_SENSOR_TYPE_Pos		0
#define	DSP_CTRL_METER_TYPE_SENSOR_TYPE_Mask	(0xFFF<<DSP_CTRL_METER_TYPE_SENSOR_TYPE_Pos)

#define	DSP_CTRL_M_MV_Pos						0
#define	DSP_CTRL_M_MV_Mask						(0xFFF<<DSP_CTRL_M_MV_Pos)

#define	DSP_CTRL_ATS_CTRL_20_23_I0_ON_Pos		0
#define	DSP_CTRL_ATS_CTRL_20_23_I0_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_20_23_I0_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_TEMPMEAS_Pos	1
#define	DSP_CTRL_ATS_CTRL_20_23_TEMPMEAS_Mask	(0x1<<DSP_CTRL_ATS_CTRL_20_23_TEMPMEAS_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_I0_GAIN_Pos		4
#define	DSP_CTRL_ATS_CTRL_20_23_I0_GAIN_Mask	(0x3<<DSP_CTRL_ATS_CTRL_20_23_I0_GAIN_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_I1_ON_Pos		8
#define	DSP_CTRL_ATS_CTRL_20_23_I1_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_20_23_I1_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_I1_GAIN_Pos		12
#define	DSP_CTRL_ATS_CTRL_20_23_I1_GAIN_Mask	(0x3<<DSP_CTRL_ATS_CTRL_20_23_I1_GAIN_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_V1_ON_Pos		16
#define	DSP_CTRL_ATS_CTRL_20_23_V1_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_20_23_I2_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_I2_ON_Pos		24
#define	DSP_CTRL_ATS_CTRL_20_23_I2_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_20_23_I2_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_20_23_I2_GAIN_Pos		28
#define	DSP_CTRL_ATS_CTRL_20_23_I2_GAIN_Mask	(0x3<<DSP_CTRL_ATS_CTRL_20_23_I2_GAIN_Pos)

#define	DSP_CTRL_ATS_CTRL_24_27_V2_ON_Pos		0
#define	DSP_CTRL_ATS_CTRL_24_27_V2_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_V2_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_I3_ON_Pos		8
#define	DSP_CTRL_ATS_CTRL_24_27_I3_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_I3_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_I3_GAIN_Pos		12
#define	DSP_CTRL_ATS_CTRL_24_27_I3_GAIN_Mask	(0x3<<DSP_CTRL_ATS_CTRL_24_27_I3_GAIN_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_V3_ON_Pos		16
#define	DSP_CTRL_ATS_CTRL_24_27_V3_ON_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_V3_ON_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_ONBIAS_Pos		24
#define	DSP_CTRL_ATS_CTRL_24_27_ONBIAS_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_ONBIAS_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_ONREF_Pos		25
#define	DSP_CTRL_ATS_CTRL_24_27_ONREF_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_ONREF_Pos)
#define	DSP_CTRL_ATS_CTRL_24_27_ONLDO_Pos		26
#define	DSP_CTRL_ATS_CTRL_24_27_ONLDO_Mask		(0x1<<DSP_CTRL_ATS_CTRL_24_27_ONLDO_Pos)

//---------------------------------------------------------
typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	FIRMWARE_VER			: 24; //
        uint32_t			: 8; //
    } BIT;
} VERSION_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	ST		: 4; //
        uint32_t			: 28; //
    } BIT;
} STATUS_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	PH_A_PRESENT	: 1; //
        uint32_t	PH_B_PRESENT	: 1; //
        uint32_t	PH_C_PRESENT	: 1; //
        uint32_t	ACTIVE_CHANNLE	: 2;
        uint32_t	FREQ_CLOCK		: 1;
        uint32_t				: 1;
        uint32_t	ATSENSE_FAIL	: 1;
        uint32_t	HARMONIC_M_CONF : 8;
        uint32_t	CREEP_DETECT_IA	: 1;
        uint32_t	CREEP_DETECT_IB	: 1;
        uint32_t	CREEP_DETECT_IC	: 1;
        uint32_t	CREEP_DETECT_P	: 1;
        uint32_t	CREEP_DETECT_Q	: 1;
        uint32_t				: 3;
        uint32_t	SAG_DET_VA		: 1;
        uint32_t	SAG_DET_VB		: 1;
        uint32_t	SAG_DET_VC		: 1;
        uint32_t	SWELL_DET_VA	: 1;
        uint32_t	SWELL_DET_VB	: 1;
        uint32_t	SWELL_DET_VC	: 1;

        uint32_t			: 2; //
    } BIT;
} STATE_FLAG_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	CAPTURE_OFFSET		: 24; //
        uint32_t	CAPTURE_STATE		: 4;
        uint32_t			: 3;
        uint32_t	CAPTURE_WRAP	: 1;
    } BIT;
} CAPTURE_STATUS_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	REF_TL_11_8			: 4; //
        uint32_t			: 4;
        uint32_t	REF_TL_7_0			: 8;
        uint32_t	TEMP_TL_11_8		: 4; //
        uint32_t			: 4;
        uint32_t	TEMP_TL_7_0			: 8;
    } BIT;
} ATsense_CAL_41_44_TYPE;

typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	REF_TH_11_8			: 4; //
        uint32_t			: 4;
        uint32_t	REF_TH_7_0			: 8;
        uint32_t	TEMP_TH_11_8		: 4; //
        uint32_t			: 4;
        uint32_t	TEMP_TH_7_0			: 8;
    } BIT;
} ATsense_CAL_45_48_TYPE;

typedef	struct
{
    VERSION_TYPE	VERSION;				//uQ32.0
    STATUS_TYPE		STATUS;
    STATE_FLAG_TYPE	STATE_FLAG;
    uint32_t	CP_ST;
    uint32_t	INTERVAL_NUM;
    uint32_t	N;
    int32_t	PH_OFFSET;			//sQ31.0
    uint32_t	FREQ;				//uQ20.12
    uint32_t	FREQ_VA;			//uQ8.24
    uint32_t	FREQ_VB;
    uint32_t	FREQ_VC;
    uint32_t	RESERVED0;
    int32_t	TEMPERATURE;		//sQ23.8
    int32_t	I_A_MAX;			//sQ1.30
    int32_t	I_B_MAX;
    int32_t	I_C_MAX;
    int32_t	I_Ni_MAX;
    int32_t	I_Nm_MAX;
    int32_t	V_A_MAX;
    int32_t	V_B_MAX;
    int32_t	V_C_MAX;
    uint32_t	RESERVED1;
    uint32_t	RESERVED2;
    uint32_t	RESERVED3;
    uint32_t	RESERVED4;
    uint32_t	RESERVED5;
    uint32_t	RESERVED6;
    uint32_t	RESERVED7;
    uint32_t	RESERVED8;
    uint32_t	RESERVED9;
    uint32_t	ZC_N_VA;
    uint32_t	ZC_N_VB;
    uint32_t	ZC_N_VC;

    ATsense_CAL_41_44_TYPE	ATsense_CAL_41_44;
    ATsense_CAL_45_48_TYPE	ATsense_CAL_45_48;
} DSP_ST_TYPE, *p_DSP_ST;

#define DSP_ST_SIZE (sizeof(DSP_ST_TYPE)/sizeof(uint32_t))
//---------------------------------------------------------
#define	DSP_ST_Halt				0
#define	DSP_ST_Reset			1
#define	DSP_ST_Init_DSP			2
#define	DSP_ST_DSP_Ready		3
#define	DSP_ST_Init_ATsens		4
#define	DSP_ST_ATsense_Ready	5
#define	DSP_ST_Ready			6
#define	DSP_ST_DSP_Settling		7
#define	DSP_ST_DSP_Running		8

//---------------------------------------------------------
typedef	struct
{
    uint64_t	I_A;			//uQ16.48
    uint64_t	I_B;
    uint64_t	I_C;
    uint64_t	I_Ni;
    uint64_t	I_Nm;
    uint64_t	I_A_F;
    uint64_t	I_B_F;
    uint64_t	I_C_F;
    uint64_t	I_Ni_F;
    uint64_t	RESERVED0;
    uint64_t	RESERVED1;
    uint64_t	RESERVED2;
    uint64_t	RESERVED3;
    uint64_t	RESERVED4;
    uint64_t	RESERVED5;
    int64_t	P_A;			//sQ15.48
    int64_t	P_B;
    int64_t	P_C;
    int64_t	P_A_F;
    int64_t	P_B_F;
    int64_t	P_C_F;
    uint64_t	RESERVED6;
    uint64_t	RESERVED7;
    uint64_t	RESERVED8;
    int64_t	Q_A;
    int64_t	Q_B;
    int64_t	Q_C;
    int64_t	Q_A_F;
    int64_t	Q_B_F;
    int64_t	Q_C_F;
    uint64_t	RESERVED9;
    uint64_t	RESERVED10;
    uint64_t	RESERVED11;
    uint64_t	V_A;			//uQ16.48
    uint64_t	V_B;
    uint64_t	V_C;
    uint64_t	RESERVED12;
    uint64_t	V_A_F;
    uint64_t	V_B_F;
    uint64_t	V_C_F;
    uint64_t	RESERVED13;
    uint64_t	V_AB;
    uint64_t	V_BC;
    uint64_t	V_CA;
    uint64_t	V_AB_F;
    uint64_t	V_BC_F;
    uint64_t	V_CA_F;
    uint64_t	RESERVED14;
    uint64_t	RESERVED15;
    uint64_t	RESERVED16;
    uint64_t	ACC_T0;			//uQ32.30
    uint64_t	ACC_T1;
    uint64_t	ACC_T2;

    uint64_t	RESERVED17;
    uint64_t	RESERVED18;

} DSP_ACC_TYPE;

typedef struct {
    uint64_t	ACC_T0;
    uint64_t	ACC_T1;
    uint64_t	ACC_T2;
} ACC_Tx_TYPE;

#define DSP_ACC_SIZE (sizeof(DSP_ACC_TYPE)/sizeof(uint64_t))

#define		FREQ_Q			12
#define		GAIN_P_Q		24
#define		GAIN_VI_Q		10
#define		RMS_DIV_G		1024		//(1<<GAIN_VI_Q)
#define		CAL_VI_Q		29
#define		CAL_PH_Q		31
#define		RMS_Q			40
#define		RMS_DIV_Q		0x10000000000       //(1<<RMS_Q)
#define		RMS_PQ_SYMB		0x8000000000000000	//p/q symbol bit
#define		RMS_HARMONIC	0x80000000
typedef	struct
{
    DSP_CTRL_TYPE	DSP_CTRL;
    DSP_ST_TYPE		DSP_ST;
    DSP_ACC_TYPE	DSP_ACC;
    DSP_HAR_TYPE	DSP_HAR;
} METROLOGY_TYPE, *p_Metrology;

typedef	struct
{
    uint32_t	line_id;
    uint32_t	mc;		//meter const =3200 imp/kwh
    uint32_t	sensortype;	// 0=CT,1=shunt,2=rogowski
    uint32_t	freq;		//frequency  *100
    uint32_t	gain_i;		//current pga
    uint32_t	k_i;		//CT current transform ratio
    uint32_t	rl;			//CT resistor load *10
    uint32_t	k_u;		//voltage divider ratio
    uint32_t	aim_va;		//aim voltage  *100
    uint32_t	aim_ia;		//aim current  *1000
    uint32_t	angle_a;	//phase a angle *100
    uint32_t	aim_vb;		//aim voltage  *100
    uint32_t	aim_ib;		//aim current  *1000
    uint32_t	angle_b;	//phase c angle *100
    uint32_t	aim_vc;		//aim voltage  *100
    uint32_t	aim_ic;		//aim current  *1000
    uint32_t	angle_c;	//phase c angle *100
    uint32_t	dsp_update_num;	//
    uint64_t	dsp_acc_ia;		//
    uint64_t	dsp_acc_ib;		//
    uint64_t	dsp_acc_ic;		//
    uint64_t	dsp_acc_in;		//
    uint64_t	dsp_acc_ua;		//
    uint64_t	dsp_acc_ub;		//
    uint64_t	dsp_acc_uc;		//
    uint64_t	dsp_acc_un;		//
    int64_t		dsp_acc_pa;
    int64_t		dsp_acc_pb;
    int64_t		dsp_acc_pc;
    int64_t		dsp_acc_qa;
    int64_t		dsp_acc_qb;
    int64_t		dsp_acc_qc;
    uint8_t		har_order;
} CAL_METER_TYPE, *p_Cal_Meter;
#define CAL_METER_SIZE (sizeof(CAL_METER_TYPE)/sizeof(uint32_t))

typedef enum
{
    Ph_A	= 1,
    Ph_B	= 2,
    Ph_C	= 3,
    Ph_N	= 4,
    Ph_T	= 5,

    V_A		= 1,
    V_B		= 2,
    V_C		= 3,
    V_N		= 4,

    I_A		= 1,
    I_B		= 2,
    I_C		= 3,
    I_N		= 4,

    V_ID	= 1,
    I_ID	= 2,
    Ph_ID	= 3

} Ph_V_I_ID_TYPE;
typedef enum
{
    CT      = 0,
    SHUNT   = 1,
    ROGOWSKI= 2,
    N_TYPE  = 0xFF
} SENSOR_TYPE;
typedef	union
{
    uint32_t	WORD;
    struct
    {
        uint32_t	pa_dir		: 1;
        uint32_t	pb_dir		: 1;
        uint32_t	pc_dir		: 1;
        uint32_t	pt_dir		: 1;
        uint32_t	qa_dir		: 1;
        uint32_t	qb_dir		: 1;
        uint32_t	qc_dir		: 1;
        uint32_t	qt_dir		: 1;

        uint32_t	sag_a		: 1;
        uint32_t	sag_b		: 1;
        uint32_t	sag_c		: 1;
        uint32_t			: 1;
        uint32_t	swell_a		: 1;
        uint32_t	swell_b		: 1;
        uint32_t	swell_c		: 1;

        uint32_t	pa_rev		: 1;
        uint32_t	pb_rev		: 1;
        uint32_t	pc_rev		: 1;
        uint32_t	pt_rev		: 1;
        uint32_t		    : 13;
    } BIT;
} AFE_ST_TYPE;

typedef	enum
{
    PEnergy	= 0,
    QEnergy	= 1,
    AbsAdd	= 0,
    AlgAdd	= 1
} M_ENERGY_TYPE;

typedef enum
{
    Ua = 0,
    Ub,
    Uc,
    Ia = 3,
    Ib,
    Ic,
    Pt = 6,
    Pa,
    Pb,
    Pc,
    Qt = 10,
    Qa,
    Qb,
    Qc,
    St = 14,
    Sa,
    Sb,
    Sc,
    Freq = 18,
    AngleA = 19,
    AngleB,
    AngleC
} RMS_TYPE;

#define	RMS_NUM_MAX		22
typedef	struct
{
    uint32_t	updataflag;
    uint32_t	energy;
    AFE_ST_TYPE	ST;
    uint32_t	RMS[RMS_NUM_MAX];
} AFE_TYPE;
#define AFE_SIZE (sizeof(AFE_TYPE)/sizeof(uint32_t))

typedef struct
{
  DSP_ST_TYPE STATUS;
  DSP_ACC_TYPE ACC;
  struct timespec TIMESTAMP;
} DSP_READING_TYPE;

//-------- DSP_CTRL : (DSP_CTRL Offset: 0x00) Control Register ------------//
#define		IsEn	(0x1u)
#define		IsDis	(0x0u)

//-------- STATE_CTRL : ( Offset: 0x00) Control Register ------------------//
#define DSP_CTRL_ST_CTRL_Pos		0
#define	DSP_CTRL_ST_CTRL_RST 	(0x0u)// << DSP_CTRL_ST_CTRL_Pos) //
#define	DSP_CTRL_ST_CTRL_INIT 	(0x1u)// << DSP_CTRL_ST_CTRL_Pos) //
#define	DSP_CTRL_ST_CTRL_RUN 	(0x2u)// << DSP_CTRL_ST_CTRL_Pos) //
#define	DSP_CTRL_ST_CTRL_HOLD 	(0x3u)// << DSP_CTRL_ST_CTRL_Pos) //

//-------- FEATURE_CTRL0 : ( Offset: 0x04) Control Register ---------------//
#define DSP_CTRL_FEATURE_CTRL0_ACC_BUF_EN_Pos		0
#define	DSP_CTRL_FEATURE_CTRL0_ACC_BUF_EN_DIS 	(0x0u)// << DSP_CTRL_FEATURE_CTRL0_ACC_BUF_EN_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_ACC_BUF_EN_En 	(0x1u)// << DSP_CTRL_FEATURE_CTRL0_ACC_BUF_EN_Pos) //
#define DSP_CTRL_FEATURE_CTRL0_SYNCH_Pos		4
#define	DSP_CTRL_FEATURE_CTRL0_SYNCH_BaseOnACT 	(0x0u)// << DSP_CTRL_FEATURE_CTRL0_SYNCH_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_SYNCH_BaseOnA	(0x1u)// << DSP_CTRL_FEATURE_CTRL0_SYNCH_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_SYNCH_BaseOnB 	(0x2u)// << DSP_CTRL_FEATURE_CTRL0_SYNCH_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_SYNCH_BaseOnC	(0x3u)// << DSP_CTRL_FEATURE_CTRL0_SYNCH_Pos) //
#define DSP_CTRL_FEATURE_CTRL0_DETNET_Pos		6
#define	DSP_CTRL_FEATURE_CTRL0_DETNET_NET 		(0x0u)// << DSP_CTRL_FEATURE_CTRL0_DETNET_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_DETNET_ABS		(0x1u)// << DSP_CTRL_FEATURE_CTRL0_DETNET_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_DETNET_DEL 		(0x2u)// << DSP_CTRL_FEATURE_CTRL0_DETNET_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_DETNET_GEN		(0x3u)// << DSP_CTRL_FEATURE_CTRL0_DETNET_Pos) //
#define DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_Pos		8
#define	DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_DIS 	(0x0u)// << DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_A		(0x1u)// << DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_B 		(0x2u)// << DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_C		(0x3u)// << DSP_CTRL_FEATURE_CTRL0_MISSING_PHASE_Pos) //

#define DSP_CTRL_FEATURE_CTRL0_MAX_INT_SELECT_Pos		11
#define	DSP_CTRL_FEATURE_CTRL0_MAX_INT_SELECT_1S 	(0x0u)// << DSP_CTRL_FEATURE_CTRL0_MAX_INT_SELECT_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_MAX_INT_SELECT_NS	(0x1u)// << DSP_CTRL_FEATURE_CTRL0_MAX_INT_SELECT_Pos) //
//-------- FEATURE_CTRL1 : ( Offset: 0x08) Control Register ---------------//
#define DSP_CTRL_FEATURE_CTRL1_CREEP_EN_Pos		0
#define	DSP_CTRL_FEATURE_CTRL1_CREEP_EN_DIS 	(0x0u)// << DSP_CTRL_FEATURE_CTRL1_CREEP_EN_Pos) //
#define	DSP_CTRL_FEATURE_CTRL1_CREEP_EN_En 		(0x1u)// << DSP_CTRL_FEATURE_CTRL1_CREEP_EN_Pos) //
#define DSP_CTRL_FEATURE_CTRL1_SWELL_EN_Pos		4
#define	DSP_CTRL_FEATURE_CTRL1_SWELL_EN_DIS 	(0x0u)// << DSP_CTRL_FEATURE_CTRL1_SWELL_EN_Pos) //
#define	DSP_CTRL_FEATURE_CTRL1_SWELL_EN_EN		(0x1u)// << DSP_CTRL_FEATURE_CTRL1_SWELL_EN_Pos) //
#define DSP_CTRL_FEATURE_CTRL1_SAG_EN_Pos		5
#define	DSP_CTRL_FEATURE_CTRL1_SAG_EN_DIS 		(0x0u)// << DSP_CTRL_FEATURE_CTRL1_SAG_EN_Pos) //
#define	DSP_CTRL_FEATURE_CTRL1_SAG_EN_EN		(0x1u)// << DSP_CTRL_FEATURE_CTRL1_SAG_EN_Pos) //
#define DSP_CTRL_FEATURE_CTRL1_HARMONIC_m_REQ_Pos		8
#define	DSP_CTRL_FEATURE_CTRL1_HARMONIC_m_REQ(value) 	((value) << DSP_CTRL_FEATURE_CTRL1_HARMONIC_m_REQ_Pos) //
#define DSP_CTRL_FEATURE_CTRL0_HARMONIC_EN_Pos		15
#define	DSP_CTRL_FEATURE_CTRL0_HARMONIC_EN_DIS 		(0x0u)// << DSP_CTRL_FEATURE_CTRL0_HARMONIC_EN_Pos) //
#define	DSP_CTRL_FEATURE_CTRL0_HARMONIC_EN_EN		(0x1u)// << DSP_CTRL_FEATURE_CTRL0_HARMONIC_EN_Pos) //
//---------- METER_TYPE : ( Offset: 0x10) Control Register ---------------//
#define SENSOR_TYPE_IA_Pos		0
#define	SENSOR__TYPE_RogowsKi	2

#define SERVICE_TYPE_Pos		24
#define	SERVICE_TYPE_3P4W_Y		0
#define	SERVICE_TYPE_3P3W_Y		1
#define	SERVICE_TYPE_3P4W_T		2
#define	SERVICE_TYPE_3P3W_T		3
#define	SERVICE_TYPE_3P4W_N		4
#define	SERVICE_TYPE_3P3W_N		5
#define	SERVICE_TYPE_2P5W		6
#define	SERVICE_TYPE_1P2W		7

//-------------------------------------------------------------------
#define	DSP_CTRL_FRAME_NUM_MAX	14
#define	DSP_ST_FRAME_NUM_MAX	9
#define	DSP_ACC_FRAME_NUM_MAX	14
#define DSP_HAR_FRAME_NUM_MAX	3

//-------------------------------------------------------------------
#define METROLOGY_IOCTL_RESET    0
#define METROLOGY_IOCTL_SETCFG   1
#define METROLOGY_IOCTL_SETACC   2
#define METROLOGY_IOCTL_GETACC   3
#define METROLOGY_IOCTL_RUN      4
#define METROLOGY_IOCTL_SHUTDOWN 5
#define METROLOGY_IOCTL_APPLYCFG 6

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* __CONFIGS_OMEGA_INCLUDE_METROLOGY_H */