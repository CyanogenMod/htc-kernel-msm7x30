#ifndef _LINUX_ATMEL_H
#define _LINUX_ATMEL_H

#include <linux/bitops.h>

#define ATMEL_QT602240_NAME "atmel_qt602240"
#define ATMEL_MXT224E_NAME "atmel_mxt224e"

#define INFO_BLK_FID                            0
#define INFO_BLK_VID                            1
#define INFO_BLK_VER                            2
#define INFO_BLK_BUILD                          3
#define INFO_BLK_XSIZE                          4
#define INFO_BLK_YSIZE                          5
#define INFO_BLK_OBJS                           6

#define OBJ_TABLE_TYPE                          0
#define OBJ_TABLE_LSB                           1
#define OBJ_TABLE_MSB                           2
#define OBJ_TABLE_SIZE                          3
#define OBJ_TABLE_INSTANCES                     4
#define OBJ_TABLE_RIDS                          5

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23                       23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u
#define SPT_PROTOTYPE_T35                         35u /* 224E, fw 1.1.01 */
#define DIAGNOSTIC_T37                            37u
#define PROCI_GRIPSUPPRESSION_T40                 40u /* 224E */
#define PROCI_TOUCHSUPPRESSION_T42                42u /* 224E */
#define SPT_MESSAGECOUNT_T44                      44u /* 224E */
#define SPT_CTECONFIG_T46                         46u /* 224E */
#define PROCI_STYLUS_T47                          47u /* 224E */
#define PROCG_NOISESUPPRESSION_T48                48u /* 224E */
#define PROCI_ADAPTIVETHRESHOLD_T55               55u /* 224E, FW v1.1 */
#define EXTRA_NOISE_SUPPRESSION_T58		  58u /* 224E, FW v1.1.AA */

#define T37_PAGE_SIZE                           128

#define T37_TCH_FLAG_SIZE                       80
#define T37_TCH_FLAG_IDX                        0
#define T37_ATCH_FLAG_IDX                       40

#define T37_MODE                                0
#define T37_PAGE                                1
#define T37_DATA                                2 /* n bytes */

#define T37_PAGE_NUM0                           0
#define T37_PAGE_NUM1                           1
#define T37_PAGE_NUM2                           2
#define T37_PAGE_NUM3                           3

#define MSG_RID                                 0

#define T6_CFG_RESET                            0
#define T6_CFG_BACKUPNV                         1
#define T6_CFG_CALIBRATE                        2
#define T6_CFG_REPORTALL                        3
/* Reserved */
#define T6_CFG_DIAG                             5

#define T6_CFG_DIAG_CMD_PAGEUP                  0x01
#define T6_CFG_DIAG_CMD_PAGEDOWN                0x02
#define T6_CFG_DIAG_CMD_DELTAS                  0x10
#define T6_CFG_DIAG_CMD_REF                     0x11
#define T6_CFG_DIAG_CMD_CTE                     0x31
#define T6_CFG_DIAG_CMD_TCH                     0xF3

#define T6_MSG_STATUS                           1
#define T6_MSG_CHECKSUM                         2 /* three bytes */

#define T6_MSG_STATUS_COMSERR                   BIT(2)
#define T6_MSG_STATUS_CFGERR                    BIT(3)
#define T6_MSG_STATUS_CAL                       BIT(4)
#define T6_MSG_STATUS_SIGERR                    BIT(5)
#define T6_MSG_STATUS_OFL                       BIT(6)
#define T6_MSG_STATUS_RESET                     BIT(7)

#define T7_CFG_IDLEACQINT                       0
#define T7_CFG_ACTVACQINT                       1
#define T7_CFG_ACTV2IDLETO                      2

#define T8_CFG_CHRGTIME                         0
/* Reserved */
#define T8_CFG_TCHDRIFT                         2
#define T8_CFG_DRIFTST                          3
#define T8_CFG_TCHAUTOCAL                       4
#define T8_CFG_SYNC                             5
#define T8_CFG_ATCHCALST                        6
#define T8_CFG_ATCHCALSTHR                      7
#define T8_CFG_ATCHFRCCALTHR                    8 /* FW v2.x */
#define T8_CFG_ATCHFRCCALRATIO                  9 /* FW v2.x */

#define T9_CFG_CTRL                             0
#define T9_CFG_XORIGIN                          1
#define T9_CFG_YORIGIN                          2
#define T9_CFG_XSIZE                            3
#define T9_CFG_YSIZE                            4
#define T9_CFG_AKSCFG                           5
#define T9_CFG_BLEN                             6
#define T9_CFG_TCHTHR                           7
#define T9_CFG_TCHDI                            8
#define T9_CFG_ORIENT                           9
#define T9_CFG_MRGTIMEOUT                       10
#define T9_CFG_MOVHYSTI                         11
#define T9_CFG_MOVHYSTN                         12
#define T9_CFG_MOVFILTER                        13
#define T9_CFG_NUMTOUCH                         14
#define T9_CFG_MRGHYST                          15
#define T9_CFG_MRGTHR                           16
#define T9_CFG_AMPHYST                          17
#define T9_CFG_XRANGE                           18 /* two bytes */
#define T9_CFG_YRANGE                           20 /* two bytes */
#define T9_CFG_XLOCLIP                          22
#define T9_CFG_XHICLIP                          23
#define T9_CFG_YLOCLIP                          24
#define T9_CFG_YHICLIP                          25
#define T9_CFG_XEDGECTRL                        26
#define T9_CFG_XEDGEDIST                        27
#define T9_CFG_YEDGECTRL                        28
#define T9_CFG_YEDGEDIST                        29
#define T9_CFG_JUMPLIMIT                        30
#define T9_CFG_TCHHYST                          31 /* FW v2.x */
#define T9_CFG_XPITCH				32 /* 224E */
#define T9_CFG_YPITCH				33
#define T9_CFG_NEXTTCHDI			34

#define T9_MSG_STATUS                           1
#define T9_MSG_XPOSMSB                          2
#define T9_MSG_YPOSMSB                          3
#define T9_MSG_XYPOSLSB                         4
#define T9_MSG_TCHAREA                          5
#define T9_MSG_TCHAMPLITUDE                     6
#define T9_MSG_TCHVECTOR                        7

#define T9_MSG_STATUS_UNGRIP                    BIT(0) /* FW v2.x */
#define T9_MSG_STATUS_SUPPRESS                  BIT(1)
#define T9_MSG_STATUS_AMP                       BIT(2)
#define T9_MSG_STATUS_VECTOR                    BIT(3)
#define T9_MSG_STATUS_MOVE                      BIT(4)
#define T9_MSG_STATUS_RELEASE                   BIT(5)
#define T9_MSG_STATUS_PRESS                     BIT(6)
#define T9_MSG_STATUS_DETECT                    BIT(7)

#define T20_CFG_CTRL                            0
#define T20_CFG_XLOGRIP                         1
#define T20_CFG_XHIGRIP                         2
#define T20_CFG_YLOGRIP                         3
#define T20_CFG_YHIGRIP                         4
#define T20_CFG_MAXTCHS                         5
/* Reserved */
#define T20_CFG_SZTHR1                          7
#define T20_CFG_SZTHR2                          8
#define T20_CFG_SHPTHR1                         9
#define T20_CFG_SHPTHR2                         10
#define T20_CFG_SHPEXTTO                        11

#define T20_MSG_STATUS                          1

#define T20_MSG_STATUS_FACESUP                  BIT(0)

#define T22_CFG_CTRL                            0
/* Reserved */
#define T22_CFG_GCAFUL                          3 /* two bytes */
#define T22_CFG_GCAFLL                          5 /* two bytes */
#define T22_CFG_ACTVGCAFVALID                   7
#define T22_CFG_NOISETHR                        8
/* Reserved */
#define T22_CFG_FREQHOPSCALE                    10
#define T22_CFG_FREQ                            11 /* five bytes */
#define T22_CFG_IDLEGCAFVAILD                   16

#define T22_MSG_STATUS                          1
#define T22_MSG_GCAFDEPTH                       2
#define T22_MSG_FREQINDEX                       3

#define T22_MSG_STATUS_FHCHG                    BIT(0)
#define T22_MSG_STATUS_GCAFERR                  BIT(2)
#define T22_MSG_STATUS_FHERR                    BIT(3)
#define T22_MSG_STATUS_GCAFCHG                  BIT(4)

#define T19_CFG_CTRL                            0
#define T19_CFG_REPORTMASK                      1
#define T19_CFG_DIR                             2
#define T19_CFG_INTPULLUP                       3
#define T19_CFG_OUT                             4
#define T19_CFG_WAKE                            5
#define T19_CFG_PWM                             6
#define T19_CFG_PERIOD                          7
#define T19_CFG_DUTY0                           8
#define T19_CFG_DUTY1                           9
#define T19_CFG_DUTY2                           10
#define T19_CFG_DUTY3                           11
#define T19_CFG_TRIGGER0                        12
#define T19_CFG_TRIGGER1                        13
#define T19_CFG_TRIGGER2                        14
#define T19_CFG_TRIGGER3                        15

#define T19_CFG_CTRL_ENABLE                     BIT(0)
#define T19_CFG_CTRL_RPTEN                      BIT(1)
#define T19_CFG_CTRL_FORCERPT                   BIT(2)

#define T19_MSG_STATUS                          1

#define T25_CFG_CTRL                            0
#define T25_CFG_CMD                             1

#define T25_MSG_STATUS                          1
#define T25_MSG_INFO                            2 /* five bytes */

#define T28_CFG_CTRL                            0
#define T28_CFG_CMD                             1
#define T28_CFG_MODE                            2
#define T28_CFG_IDLEGCAFDEPTH                   3
#define T28_CFG_ACTVGCAFDEPTH                   4
#define T28_CFG_VOLTAGE                         5

#define T28_CFG_MODE0_X                         16
#define T28_CFG_MODE0_Y                         14

#define T28_MSG_STATUS                          1

/* T40, T42, T47, T48 and T46 are supported by 224E */
#define T40_CFG_CTRL                            0
#define T40_CFG_XLOGRIP                         1
#define T40_CFG_XHIGRIP                         2
#define T40_CFG_YLOGRIP                         3
#define T40_CFG_YHIGRIP                         4

#define T42_CFG_CTRL                            0
#define T42_CFG_APPRTHR                         1
#define T42_CFG_MAXAPPRAREA                     2
#define T42_CFG_MAXTCHAREA                      3
#define T42_CFG_SUPSTRENGTH                     4
#define T42_CFG_SUPEXTTO                        5
#define T42_CFG_MAXNUMTCHS                      6
#define T42_CFG_SHAPESTRENGTH                   7

#define T42_MSG_STATUS                          1

#define T47_CFG_CTRL                            0
#define T47_CFG_CONTMIN                         1
#define T47_CFG_CONTMAX                         2
#define T47_CFG_STABILITY                       3
#define T47_CFG_MAXTCHAREA                      4
#define T47_CFG_AMPLTHR                         5
#define T47_CFG_STYSHAPE                        6
#define T47_CFG_HOVERSUP                        7
#define T47_CFG_CONFTHR                         8
#define T47_CFG_SYNCSPERX                       9

#define T48_CFG_CTRL                            0
#define T48_CFG_CFG                             1
#define T48_CFG_CALCFG                          2
#define T48_CFG_BASEFREQ                        3
#define T48_CFG_FREQ                            4 /* four bytes */
#define T48_CFG_MFFREQ                          8 /* two bytes */
#define T48_CFG_NLGAIN                          10
#define T48_CFG_NLTHR                           11
#define T48_CFG_GCLIMIT                         12
#define T48_CFG_GCACTVINVLDADCS                 13
#define T48_CFG_GCIDLEINVLDADCS                 14
#define T48_CFG_GCINVALIDTHR                    15 /* two bytes */
#define T48_CFG_GCMAXADCSPERX                   17
#define T48_CFG_GCLIMITMIN                      18
#define T48_CFG_GCLIMITMAX                      19
#define T48_CFG_GCCOUNTMINTGT                   20 /* two bytes */
#define T48_CFG_MFINVLDDIFFTHR                  22
#define T48_CFG_MFINCADCSPXTHR                  23 /* two bytes */
#define T48_CFG_MFERRORTHR                      25 /* two bytes */
#define T48_CFG_SELFREQMAX                      27
#define T48_CFG_NOCALCFG                        28 /* FW v1.1 */
/* Reserved */
#define T48_CFG_T9SETTINGS                      34

#define T48_MSG_STATUS                          1
#define T48_MSG_ADCSPERX                        2
#define T48_MSG_FREQ                            3
#define T48_MSG_STATE                           4
#define T48_MSG_NOISE_LV                        5
#define T48_MSG_NLTHR				6

#define T48_MSG_STATUS_FREQCHG                  BIT(0)
#define T48_MSG_STATUS_APXCHG                   BIT(1)
#define T48_MSG_STATUS_ALGOERR                  BIT(2)
#define T48_MSG_STATUS_STATCHG                  BIT(4)
#define T48_MSG_STATUS_NLVLCGH			BIT(5)

#define T48_MSG_STATE_OFF                       0
#define T48_MSG_STATE_SEARCH                    1
#define T48_MSG_STATE_GC                        2
#define T48_MSG_STATE_GC_ERR                    3
#define T48_MSG_STATE_MF                        4
#define T48_MSG_STATE_MF_ERR                    5
#define T48_MSG_STATE_NORMAL                    6

#define T46_CFG_CTRL                            0
#define T46_CFG_MODE                            1
#define T46_CFG_IDLESYNCSPERX                   2
#define T46_CFG_ACTVSYNCSPERX                   3
#define T46_CFG_ADCSPERSYNC                     4
#define T46_CFG_PULSESPERADC                    5
#define T46_CFG_XSLEW                           6
#define T46_CFG_SYNCDELAY                       7 /* two bytes */

#define T46_CFG_MODE0_X                         16
#define T46_CFG_MODE0_Y                         14

#define T46_MSG_STATUS                          1

/* 224E 1.1.0.1 */
#define T35_CFG_MAXTCHTHR			0
#define T35_CFG_MAXNLTHR			1
#define T35_CFG_MAXDI				2
#define T35_CFG_MAXFILTER			3
#define T35_CFG_THRCHTHR			4

/* 224E 1.1.AA */
#define T58_CFG_INCTCHTHR			0
#define T58_CFG_MAXNLTHR			1
#define T58_CFG_MAXDI				2
#define T58_CFG_MAXFILTER			3
#define T58_CFG_THRCHTHR			4

/* cable_config[] of atmel_i2c_platform_data */
/* config[] of atmel_config_data */
#define CB_TCHTHR                               0
#define CB_NOISETHR                             1
#define CB_IDLEGCAFDEPTH                        2
#define CB_ACTVGCAFDEPTH                        3
#define CB_NLTHR				1 /* 224E */
#define CB_IDLESYNCSPERX                        2 /* 224E */
#define CB_ACTVSYNCSPERX                        3 /* 224E */
#define CB_SELFREQMAX				4 /* 224E */
#define CB_BASEFREQ				5
#define CB_TCHDI				6 /* 224E */
#define CB_NEXTTCHDI				7 /* 224E */
#define CB_INCTCHTHR				8 /* 224E 1.1.AA*/
#define CB_MAXNLTHR				9
#define CB_MINTCHDI				10
#define CB_MINMOVFILTER				11
#define CB_TCHTHRCHGTHR				12

#define WLC_IDLEACQINT                          0
#define WLC_ACTVACQINT                          1
#define WLC_ACTV2IDLETO                         2
#define WLC_TCHTHR                              3
#define WLC_NOISETHR                            4
#define WLC_IDLEGCAFDEPTH                       5
#define WLC_ACTVGCAFDEPTH                       6

#define NC_TCHTHR                               0
#define NC_TCHDI                                1
#define NC_NOISETHR                             2

/* filter_level */
#define FL_XLOGRIPMIN                           0
#define FL_XLOGRIPMAX                           1
#define FL_XHIGRIPMIN                           2
#define FL_XHIGRIPMAX                           3

/* workaround */
#define TW_SHIFT				1

/* report 2 kind of finger data */
#define REPORT_BOTH_DATA			1

struct info_id_t {
	uint8_t family_id;
	uint8_t variant_id;
	uint8_t version;
	uint8_t build;
	uint8_t matrix_x_size;
	uint8_t matrix_y_size;
	uint8_t num_declared_objects;
};

struct object_t {
	uint8_t object_type;
	uint16_t i2c_address;
	uint8_t size;
	uint8_t instances;
	uint8_t num_report_ids;
	uint8_t report_ids;
};

struct atmel_virtual_key {
	int keycode;
	int range_min;
	int range_max;
};

struct atmel_finger_data {
	int x;
	int y;
	int w;
	int z;
};

struct atmel_i2c_platform_data {
	uint16_t version;
	uint16_t source;
	uint16_t build;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	int gpio_irq;
	int gpio_rst;
	int (*power)(int on);
	uint8_t unlock_attr;
	uint8_t report_both;
	int8_t config_T6[6];
	int8_t config_T7[3];
	int8_t config_T8[10];
	int8_t config_T9[35];
	int8_t config_T15[11];
	int8_t config_T18[2];
	int8_t config_T19[16];
	int8_t config_T20[12];
	int8_t config_T22[17];
	int8_t config_T23[15];
	int8_t config_T24[19];
	int8_t config_T25[14];
	int8_t config_T27[7];
	int8_t config_T28[6];
	int8_t config_T35[11];
	int8_t config_T40[5];
	int8_t config_T42[8];
	int8_t config_T46[9];
	int8_t config_T47[10];
	int8_t config_T48[54];
	int8_t config_T55[4];
	int8_t config_T58[11];
	uint8_t object_crc[3];
	int8_t cable_config[6];
	int8_t cable_config_T7[3];
	int8_t cable_config_T8[10];
	int8_t cable_config_T9[35];
	int8_t cable_config_T22[17];
	int8_t cable_config_T28[6];
	int8_t wlc_config[7];
	uint8_t wlc_freq[5];
	int8_t noise_config[3];
	uint8_t call_tchthr[2];
	uint8_t locking_config[1];
	uint16_t filter_level[4];
	uint8_t GCAF_level[5];
	uint8_t mferr_config[13];
	uint8_t noiseLine_config[8];
	uint8_t workaround;
};

struct atmel_config_data {
	int8_t config[6];
	int8_t *config_T7;
	int8_t *config_T8;
	int8_t *config_T9;
	int8_t *config_T22;
	int8_t *config_T28;
	int8_t *config_T35;
	int8_t *config_T40;
	int8_t *config_T42;
	int8_t *config_T46;
	int8_t *config_T48;
	int8_t *config_T55;
	int8_t *config_T58;
};

#endif

