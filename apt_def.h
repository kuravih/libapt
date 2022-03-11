#ifndef __APT_DEF_H__
#define __APT_DEF_H__


// page 41, 42
enum class APT_CHANNEL : uint8_t {
  CHANNEL_1 = 0x01,
  CHANNEL_2 = 0x02,
  CHANNEL_3 = 0x04,
  CHANNEL_4 = 0x08,
  CHANNEL_N_CHANNELS
};
#define APT_CHANNEL_LABELS {"CHANNEL_INVALID_0",\
                           "CHANNEL_1",\
                           "CHANNEL_2",\
                           "CHANNEL_INVALID_3",\
                           "CHANNEL_3",\
                           "CHANNEL_INVALID_4","CHANNEL_INVALID_5","CHANNEL_INVALID_6",\
                           "CHANNEL_4"}

// page 42
enum class APT_STATE : uint8_t {
  STATE_ENABLE = 0x01,
  STATE_DISABLE = 0x02,
  STATE_N_STATES
};
#define APT_STATE_LABELS {"STATE_INVALID_0",\
                          "STATE_ENABLE",\
                          "STATE_DISABLE"}

// page 184
enum class PZ_VOLTAGE_RANGE : uint8_t {
  VOLTAGE_RANGE_75V = 0x01,
  VOLTAGE_RANGE_100V,
  VOLTAGE_RANGE_150V,
  VOLTAGE_RANGE_N_RANGES
};
#define PZ_VOLTAGE_RANGE_LABELS {"VOLTAGE_RANGE_INVALID_0",\
                                 "VOLTAGE_RANGE_75V",\
                                 "VOLTAGE_RANGE_100V",\
                                 "VOLTAGE_RANGE_150V"}

// page 184
enum class PZ_ANALOG_INPUT_SOURCE : uint8_t {
  ANALOG_INPUT_SOURCE_A = 0x01,
  ANALOG_INPUT_SOURCE_B,
  ANALOG_INPUT_SOURCE_EXTSIG_SMA,
  ANALOG_INPUT_SOURCE_N_SOURCES
};
#define PZ_ANALOG_INPUT_SOURCE_LABELS {"ANALOG_INPUT_SOURCE_INVALID_0",\
                                       "ANALOG_INPUT_SOURCE_A",\
                                       "ANALOG_INPUT_SOURCE_B",\
                                       "ANALOG_INPUT_SOURCE_EXTSIG_SMA"}

// page 160
enum class PZ_INPUT_VOLTAGE_SOURCE : uint8_t {
  INPUT_VOLTAGE_SOURCE_SW_ONLY = 0x00,
  INPUT_VOLTAGE_SOURCE_EXT_SIG = 0x01,
  INPUT_VOLTAGE_SOURCE_POT = 0x02,
  INPUT_VOLTAGE_SOURCE_N_SOURCES
};
#define PZ_INPUT_VOLTAGE_SOURCE_LABELS {"INPUT_VOLTS_SRC_SW_ONLY",\
                                        "INPUT_VOLTS_SRC_EXT_SIG",\
                                        "INPUT_VOLTS_SRC_POT"}

// page 156
enum class PZ_POSITION_CONTROL_MODE : uint8_t {
  POSITION_CONTROL_MODE_OPEN_LOOP = 0x01,
  POSITION_CONTROL_MODE_CLOSED_LOOP,
  POSITION_CONTROL_MODE_OPEN_LOOP_SMOOTH,
  POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH,
  POSITION_CONTROL_MODE_N_MODES
};
#define PZ_POSITION_CONTROL_MODE_LABELS {"POSITION_CONTROL_MODE_INVALID_0",\
                                         "POSITION_CONTROL_MODE_OPEN_LOOP",\
                                         "POSITION_CONTROL_MODE_CLOSED_LOOP",\
                                         "POSITION_CONTROL_MODE_OPEN_LOOP_SMOOTH",\
                                         "POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH"}

#define APT_MSMSG_TIMEOUT_MS 100

#define APT_MGMSG_SRC_DEST_HOST 0x01
#define APT_MGMSG_SRC_DEST_RACK 0x11
#define APT_MGMSG_SRC_DEST_BAY0 0x21
#define APT_MGMSG_SRC_DEST_BAY1 0x22
#define APT_MGMSG_SRC_DEST_BAY2 0x23
#define APT_MGMSG_SRC_DEST_BAY3 0x24
#define APT_MGMSG_SRC_DEST_BAY4 0x25
#define APT_MGMSG_SRC_DEST_BAY5 0x26
#define APT_MGMSG_SRC_DEST_BAY6 0x27
#define APT_MGMSG_SRC_DEST_BAY7 0x28
#define APT_MGMSG_SRC_DEST_BAY8 0x29
#define APT_MGMSG_SRC_DEST_BAY9 0x2A
#define APT_MGMSG_SRC_DEST_GENERIC_USB 0x50

#define APT_MGMSG_LONG_DATA 0x80

// Generic System Control Messages // page 40
#define APT_MGMSG_MOD_IDENTIFY 0x0223 // page 41
#define APT_MGMSG_MOD_SET_CHANENABLESTATE 0x0210 // page 42,43
#define APT_MGMSG_MOD_REQ_CHANENABLESTATE 0x0211 // page 42,43
#define APT_MGMSG_MOD_GET_CHANENABLESTATE 0x0212 // page 42,43
#define APT_MGMSG_HW_DISCONNECT 0x0002 // page 44
#define APT_MGMSG_HW_RESPONSE 0x0080 // page 44
#define APT_MGMSG_HW_RICHRESPONSE 0x0081 // page 45
#define APT_MGMSG_HW_START_UPDATEMSGS 0x0011 // page 46
#define APT_MGMSG_HW_STOP_UPDATEMSGS 0x0012 // page 46
#define APT_MGMSG_HW_REQ_INFO 0x0005 // page 47,48
#define APT_MGMSG_HW_GET_INFO 0x0006 // page 47,48
#define APT_MGMSG_RACK_REQ_BAYUSED 0x0060 // page 49
#define APT_MGMSG_RACK_GET_BAYUSED 0x0061 // page 49
#define APT_MGMSG_HUB_REQ_BAYUSED 0x0065 // page 50
#define APT_MGMSG_HUB_GET_BAYUSED 0x0066 // page 50
#define APT_MGMSG_RACK_REQ_STATUSBITS 0x0226 // page 51
#define APT_MGMSG_RACK_GET_STATUSBITS 0x0227 // page 51
#define APT_MGMSG_RACK_SET_DIGOUTPUTS 0x0228 // page 52
#define APT_MGMSG_RACK_REQ_DIGOUTPUTS 0x0229 // page 52
#define APT_MGMSG_RACK_GET_DIGOUTPUTS 0x0230 // page 52
#define APT_MGMSG_MOD_SET_DIGOUTPUTS 0x0213 // page 53
#define APT_MGMSG_MOD_REQ_DIGOUTPUTS 0x0214 // page 53
#define APT_MGMSG_MOD_GET_DIGOUTPUTS 0x0215 // page 53
#define APT_MGMSG_HW_SET_KCUBEMMILOCK 0x0250 // page 54
#define APT_MGMSG_HW_REQ_KCUBEMMILOCK 0x0251 // page 54
#define APT_MGMSG_HW_GET_KCUBEMMILOCK 0x0252 // page 54
#define APT_MGMSG_RESTOREFACTORYSETTINGS 0x0686 // page 55

// Motor Control Messages // page 56
#define APT_MGMSG_HW_YES_FLASH_PROGRAMMING 0x0017 // page 57
#define APT_MGMSG_HW_NO_FLASH_PROGRAMMING 0x0018 // page 57
#define APT_MGMSG_MOT_SET_POSCOUNTER 0x0410 // page 58
#define APT_MGMSG_MOT_REQ_POSCOUNTER 0x0411 // page 58
#define APT_MGMSG_MOT_GET_POSCOUNTER 0x0412 // page 58
#define APT_MGMSG_MOT_SET_ENCCOUNTER 0x0409 // page 59,60
#define APT_MGMSG_MOT_REQ_ENCCOUNTER 0x040A // page 59,60
#define APT_MGMSG_MOT_GET_ENCCOUNTER 0x040B // page 59,60
#define APT_MGMSG_MOT_SET_VELPARAMS 0x0413 // page 61,62
#define APT_MGMSG_MOT_REQ_VELPARAMS 0x0414 // page 61,62
#define APT_MGMSG_MOT_GET_VELPARAMS 0x0415 // page 61,62
#define APT_MGMSG_MOT_SET_JOGPARAMS 0x0416 // page 63,64
#define APT_MGMSG_MOT_REQ_JOGPARAMS 0x0417 // page 63,64
#define APT_MGMSG_MOT_GET_JOGPARAMS 0x0418 // page 63,64
#define APT_MGMSG_MOT_REQ_ADCINPUTS 0x042B // page 65
#define APT_MGMSG_MOT_GET_ADCINPUTS 0x042C // page 65
#define APT_MGMSG_MOT_SET_POWERPARAMS 0x0426 // page 66,67
#define APT_MGMSG_MOT_REQ_POWERPARAMS 0x0427 // page 66,67
#define APT_MGMSG_MOT_GET_POWERPARAMS 0x0428 // page 66,67
#define APT_MGMSG_MOT_SET_GENMOVEPARAMS 0x043A // page 68
#define APT_MGMSG_MOT_REQ_GENMOVEPARAMS 0x043B // page 68
#define APT_MGMSG_MOT_GET_GENMOVEPARAMS 0x043C // page 68
#define APT_MGMSG_MOT_SET_MOVERELPARAMS 0x0445 // page 69
#define APT_MGMSG_MOT_REQ_MOVERELPARAMS 0x0446 // page 69
#define APT_MGMSG_MOT_GET_MOVERELPARAMS 0x0447 // page 69
#define APT_MGMSG_MOT_SET_MOVEABSPARAMS 0x0450 // page 70
#define APT_MGMSG_MOT_REQ_MOVEABSPARAMS 0x0451 // page 70
#define APT_MGMSG_MOT_GET_MOVEABSPARAMS 0x0452 // page 70
#define APT_MGMSG_MOT_SET_HOMEPARAMS 0x0440 // page 71,72
#define APT_MGMSG_MOT_REQ_HOMEPARAMS 0x0441 // page 71,72
#define APT_MGMSG_MOT_GET_HOMEPARAMS 0x0442 // page 71,72
#define APT_MGMSG_MOT_SET_LIMSWITCHPARAMS 0x0423 // page 73,74
#define APT_MGMSG_MOT_REQ_LIMSWITCHPARAMS 0x0424 // page 73,74
#define APT_MGMSG_MOT_GET_LIMSWITCHPARAMS 0x0425 // page 73,74
#define APT_MGMSG_MOT_MOVE_HOME 0x0443 // page 75
#define APT_MGMSG_MOT_MOVE_HOMED 0x0444 // page 75
#define APT_MGMSG_MOT_MOVE_RELATIVE 0x0448 // page 76,77
#define APT_MGMSG_MOT_MOVE_COMPLETED 0x0464 // page 78
#define APT_MGMSG_MOT_MOVE_ABSOLUTE 0x0453 // page 79,80
#define APT_MGMSG_MOT_MOVE_JOG 0x046A // page 81
#define APT_MGMSG_MOT_MOVE_VELOCITY 0x0457 // page 82
#define APT_MGMSG_MOT_MOVE_STOP 0x0465 // page 83
#define APT_MGMSG_MOT_MOVE_STOPPED 0x0466 // page 84
#define APT_MGMSG_MOT_SET_BOWINDEX 0x04F4 // page 85,86,87
#define APT_MGMSG_MOT_REQ_BOWINDEX 0x04F5 // page 85,86,87
#define APT_MGMSG_MOT_GET_BOWINDEX 0x04F6 // page 85,86,87
#define APT_MGMSG_MOT_SET_DCPIDPARAMS 0x04A0 // page 88,89
#define APT_MGMSG_MOT_REQ_DCPIDPARAMS 0x04A1 // page 88,89
#define APT_MGMSG_MOT_GET_DCPIDPARAMS 0x04A2 // page 88,89
#define APT_MGMSG_MOT_SET_AVMODES 0x04B3 // page 90,91
#define APT_MGMSG_MOT_REQ_AVMODES 0x04B4 // page 90,91
#define APT_MGMSG_MOT_GET_AVMODES 0x04B5 // page 90,91
#define APT_MGMSG_MOT_SET_POTPARAMS 0x04B0 // page 92,93,94
#define APT_MGMSG_MOT_REQ_POTPARAMS 0x04B1 // page 92,93,94
#define APT_MGMSG_MOT_GET_POTPARAMS 0x04B2 // page 92,93,94
#define APT_MGMSG_MOT_SET_BUTTONPARAMS 0x04B6 // page 95,96
#define APT_MGMSG_MOT_REQ_BUTTONPARAMS 0x04B7 // page 95,96
#define APT_MGMSG_MOT_GET_BUTTONPARAMS 0x04B8 // page 95,96
#define APT_MGMSG_MOT_SET_EEPROMPARAMS 0x04B9 // page 97
#define APT_MGMSG_MOT_SET_POSITIONLOOPPARAMS 0x04D7 // page 98,99,100
#define APT_MGMSG_MOT_REQ_POSITIONLOOPPARAMS 0x04D8 // page 98,99,100
#define APT_MGMSG_MOT_GET_POSITIONLOOPPARAMS 0x04D9 // page 98,99,100
#define APT_MGMSG_MOT_SET_MOTOROUTPUTPARAMS 0x04DA // page 101,102
#define APT_MGMSG_MOT_REQ_MOTOROUTPUTPARAMS 0x04DB // page 101,102
#define APT_MGMSG_MOT_GET_MOTOROUTPUTPARAMS 0x04DC // page 101,102
#define APT_MGMSG_MOT_SET_TRACKSETTLEPARAMS 0x04E0 // page 103,104,105
#define APT_MGMSG_MOT_REQ_TRACKSETTLEPARAMS 0x04E1 // page 103,104,105
#define APT_MGMSG_MOT_GET_TRACKSETTLEPARAMS 0x04E2 // page 103,104,105
#define APT_MGMSG_MOT_SET_PROFILEMODEPARAMS 0x04E3 // page 106,107
#define APT_MGMSG_MOT_REQ_PROFILEMODEPARAMS 0x04E4 // page 106,107
#define APT_MGMSG_MOT_GET_PROFILEMODEPARAMS 0x04E5 // page 106,107
#define APT_MGMSG_MOT_SET_JOYSTICKPARAMS 0x04E6 // page 108,109
#define APT_MGMSG_MOT_REQ_JOYSTICKPARAMS 0x04E7 // page 108,109
#define APT_MGMSG_MOT_GET_JOYSTICKPARAMS 0x04E8 // page 108,109
#define APT_MGMSG_MOT_SET_CURRENTLOOPPARAMS 0x04D4 // page 110,111
#define APT_MGMSG_MOT_REQ_CURRENTLOOPPARAMS 0x04D5 // page 110,111
#define APT_MGMSG_MOT_GET_CURRENTLOOPPARAMS 0x04D6 // page 110,111
#define APT_MGMSG_MOT_SET_SETTLEDCURRENTLOOPPARAMS 0x04E9 // page 112,113
#define APT_MGMSG_MOT_REQ_SETTLEDCURRENTLOOPPARAMS 0x04EA // page 112,113
#define APT_MGMSG_MOT_GET_SETTLEDCURRENTLOOPPARAMS 0x04EB // page 112,113
#define APT_MGMSG_MOT_SET_STAGEAXISPARAMS 0x04F0 // page 114,115
#define APT_MGMSG_MOT_REQ_STAGEAXISPARAMS 0x04F1 // page 114,115
#define APT_MGMSG_MOT_GET_STAGEAXISPARAMS 0x04F2 // page 114,115
#define APT_MGMSG_MOT_SET_TSTACTUATORTYPE 0x04FE // page 116
#define APT_MGMSG_MOT_GET_STATUSUPDATE 0x0481 // page 117,118
#define APT_MGMSG_MOT_REQ_STATUSUPDATE 0x0480 // page 118
#define APT_MGMSG_MOT_GET_DCSTATUSUPDATE 0x0491 // page 119,120
#define APT_MGMSG_MOT_REQ_DCSTATUSUPDATE 0x0490 // page 120
#define APT_MGMSG_MOT_ACK_DCSTATUSUPDATE 0x0492 // page 120
#define APT_MGMSG_MOT_REQ_STATUSBITS 0x0429 // page 121
#define APT_MGMSG_MOT_GET_STATUSBITS 0x042A // page 121
#define APT_MGMSG_MOT_SUSPEND_ENDOFMOVEMSGS 0x046B // page 122
#define APT_MGMSG_MOT_RESUME_ENDOFMOVEMSGS 0x046C // page 123
#define APT_MGMSG_MOT_SET_TRIGGER 0x0500 // page 124,125,126
#define APT_MGMSG_MOT_REQ_TRIGGER 0x0501 // page 124,125,126
#define APT_MGMSG_MOT_GET_TRIGGER 0x0502 // page 124,125,126
#define APT_MGMSG_MOT_SET_KCUBEMMIPARAMS 0x0520 // page 127,128,129
#define APT_MGMSG_MOT_REQ_KCUBEMMIPARAMS 0x0521 // page 127,128,129
#define APT_MGMSG_MOT_GET_KCUBEMMIPARAMS 0x0522 // page 127,128,129
#define APT_MGMSG_MOT_SET_KCUBETRIGIOCONFIG 0x0523 // page 130,131,132,133
#define APT_MGMSG_MOT_REQ_KCUBETRIGCONFIG 0x0524 // page 130,131,132,133
#define APT_MGMSG_MOT_GET_KCUBETRIGCONFIG 0x0525 // page 130,131,132,133
#define APT_MGMSG_MOT_SET_KCUBEPOSTRIGPARAMS 0x0526 // page 134,135,136,137
#define APT_MGMSG_MOT_REQ_KCUBEPOSTRIGPARAMS 0x0527 // page 134,135,136,137
#define APT_MGMSG_MOT_GET_KCUBEPOSTRIGPARAMS 0x0528 // page 134,135,136,137
#define APT_MGMSG_MOT_SET_KCUBEKSTLOOPPARAMS 0x0529 // page 138,139,140
#define APT_MGMSG_MOT_REQ_KCUBEKSTLOOPPARAMS 0x052A // page 138,139,140
#define APT_MGMSG_MOT_GET_KCUBEKSTLOOPPARAMS 0x052B // page 138,139,140

//Filter Flipper Control Messages // page 141
#define APT_MGMSG_MOT_SET_MFF_OPERPARAMS 0x0510 // page 142,143,144,145
#define APT_MGMSG_MOT_REQ_MFF_OPERPARAMS 0x0511 // page 142,143,144,145
#define APT_MGMSG_MOT_GET_MFF_OPERPARAMS 0x0512 // page 142,143,144,145


// Solenoid Control Messages // page 146
#define APT_MGMSG_MOT_SET_SOL_OPERATINGMODE 0x04C0 // page 147,148
#define APT_MGMSG_MOT_REQ_SOL_OPERATINGMODE 0x04C1 // page 147,148
#define APT_MGMSG_MOT_GET_SOL_OPERATINGMODE 0x04C2 // page 147,148
#define APT_MGMSG_MOT_SET_SOL_CYCLEPARAMS 0x04C3 // page 149,150
#define APT_MGMSG_MOT_REQ_SOL_CYCLEPARAMS 0x04C4 // page 149,150
#define APT_MGMSG_MOT_GET_SOL_CYCLEPARAMS 0x04C5 // page 149,150
#define APT_MGMSG_MOT_SET_SOL_INTERLOCKMODE 0x04C6 // page 151,152
#define APT_MGMSG_MOT_REQ_SOL_INTERLOCKMODE 0x04C7 // page 151,152
#define APT_MGMSG_MOT_GET_SOL_INTERLOCKMODE 0x04C8 // page 151,152
#define APT_MGMSG_MOT_SET_SOL_STATE 0x04CB // page 153,154
#define APT_MGMSG_MOT_REQ_SOL_STATE 0x04CC // page 153,154
#define APT_MGMSG_MOT_GET_SOL_STATE 0x04CD // page 153,154

// Piezo Control Messages // page 155
#define APT_MGMSG_PZ_SET_POSCONTROLMODE 0x0640 // page 156.157
#define APT_MGMSG_PZ_REQ_POSCONTROLMODE 0x0641 // page 156.157
#define APT_MGMSG_PZ_GET_POSCONTROLMODE 0x0642 // page 156.157
#define APT_MGMSG_PZ_SET_OUTPUTVOLTS 0x0643 // page 158
#define APT_MGMSG_PZ_REQ_OUTPUTVOLTS 0x0644 // page 158
#define APT_MGMSG_PZ_GET_OUTPUTVOLTS 0x0645 // page 158
#define APT_MGMSG_PZ_SET_OUTPUTPOS 0x0646 // page 159
#define APT_MGMSG_PZ_REQ_OUTPUTPOS 0x0647 // page 159
#define APT_MGMSG_PZ_GET_OUTPUTPOS 0x0648 // page 159
#define APT_MGMSG_PZ_SET_INPUTVOLTSSRC 0x0652 // page 160,161
#define APT_MGMSG_PZ_REQ_INPUTVOLTSSRC 0x0653 // page 160,161
#define APT_MGMSG_PZ_GET_INPUTVOLTSSRC 0x0654 // page 160,161
#define APT_MGMSG_PZ_SET_PICONSTS 0x0655 // page 162
#define APT_MGMSG_PZ_REQ_PICONSTS 0x0656 // page 162
#define APT_MGMSG_PZ_GET_PICONSTS 0x0657 // page 162
#define APT_MGMSG_PZ_REQ_PZSTATUSBITS 0x065B // page 163,164
#define APT_MGMSG_PZ_GET_PZSTATUSBITS 0x065C // page 163,164
#define APT_MGMSG_PZ_REQ_PZSTATUSUPDATE 0x0660 // pge 165,166
#define APT_MGMSG_PZ_GET_PZSTATUSUPDATE 0x0661 // pge 165,166
#define APT_MGMSG_PZ_ACK_PZSTATUSUPDATE 0x0662 // page 167
#define APT_MGMSG_PZ_SET_PPC_PIDCONSTS 0x0690 // page 168,169
#define APT_MGMSG_PZ_REQ_PPC_PIDCONSTS 0x0691 // page 168,169
#define APT_MGMSG_PZ_GET_PPC_PIDCONSTS 0x0692 // page 168,169
#define APT_MGMSG_PZ_SET_PPC_NOTCHPARAMS 0x0693 // page 170,171
#define APT_MGMSG_PZ_REQ_PPC_NOTCHPARAMS 0x0694 // page 170,171
#define APT_MGMSG_PZ_GET_PPC_NOTCHPARAMS 0x0695 // page 170,171
#define APT_MGMSG_PZ_SET_PPC_IOSETTINGS 0x0696 // page 172,173,174
#define APT_MGMSG_PZ_REQ_PPC_IOSETTINGS 0x0697 // page 172,173,174
#define APT_MGMSG_PZ_GET_PPC_IOSETTINGS 0x0698 // page 172,173,174
#define APT_MGMSG_PZ_SET_OUTPUTLUT 0x0700 // page 175,176
#define APT_MGMSG_PZ_REQ_OUTPUTLUT 0x0701 // page 175,176
#define APT_MGMSG_PZ_GET_OUTPUTLUT 0x0702 // page 175,176
#define APT_MGMSG_PZ_SET_OUTPUTLUTPARAMS 0x0703 // page 177,178,179,180
#define APT_MGMSG_PZ_REQ_OUTPUTLUTPARAMS 0x0704 // page 177,178,179,180
#define APT_MGMSG_PZ_GET_OUTPUTLUTPARAMS 0x0705 // page 177,178,179,180
#define APT_MGMSG_PZ_START_LUTOUTPUT 0x0706 // page 181
#define APT_MGMSG_PZ_STOP_LUTOUTPUT 0x0707 // page 181
#define APT_MGMSG_PZ_SET_EEPROMPARAMS 0x07D0 // page 182
#define APT_MGMSG_PZ_SET_TPZ_DISPSETTINGS 0x07D1 // page 183
#define APT_MGMSG_PZ_REQ_TPZ_DISPSETTINGS 0x07D2 // page 183
#define APT_MGMSG_PZ_GET_TPZ_DISPSETTINGS 0x07D3 // page 183
#define APT_MGMSG_PZ_SET_TPZ_IOSETTINGS 0x07D4 // page 184,185
#define APT_MGMSG_PZ_REQ_TPZ_IOSETTINGS 0x07D5 // page 184,185
#define APT_MGMSG_PZ_GET_TPZ_IOSETTINGS 0x07D6 // page 184,185
#define APT_MGMSG_PZ_SET_ZERO 0x0658 // page 186
#define APT_MGMSG_PZ_REQ_MAXTRAVEL 0x0650 // page 187
#define APT_MGMSG_PZ_GET_MAXTRAVEL 0x0651 // page 187
#define APT_MGMSG_PZ_SET_IOSETTINGS 0x0670 // page 188,189
#define APT_MGMSG_PZ_REQ_IOSETTINGS 0x0671 // page 188,189
#define APT_MGMSG_PZ_GET_IOSETTINGS 0x0672 // page 188,189
#define APT_MGMSG_PZ_SET_OUTPUTMAXVOLTS 0x0680 // page 190,191
#define APT_MGMSG_PZ_REQ_OUTPUTMAXVOLTS 0x0681 // page 190,191
#define APT_MGMSG_PZ_GET_OUTPUTMAXVOLTS 0x0682 // page 190,191
#define APT_MGMSG_PZ_SET_TPZ_SLEWRATES 0x0683 // page 192,193
#define APT_MGMSG_PZ_REQ_TPZ_SLEWRATES 0x0684 // page 192,193
#define APT_MGMSG_PZ_GET_TPZ_SLEWRATES 0x0685 // page 192,193
#define APT_MGMSG_PZ_SET_LUTVALUETYPE 0x0708 // page 194
#define APT_MGMSG_KPZ_SET_KCUBEMMIPARAMS 0x07F0 // page 195,196
#define APT_MGMSG_KPZ_REQ_KCUBEMMIPARAMS 0x07F1 // page 195,196
#define APT_MGMSG_KPZ_GET_KCUBEMMIPARAMS 0x07F2 // page 195,196
#define APT_MGMSG_KPZ_SET_KCUBETRIGIOCONFIG 0x07F3 // page 197,198,199
#define APT_MGMSG_KPZ_REQ_KCUBETRIGIOCONFIG 0x07F4 // page 197,198,199
#define APT_MGMSG_KPZ_GET_KCUBETRIGIOCONFIG 0x07F5 // page 197,198,199
#define APT_MGMSG_PZ_SET_TSG_IOSETTINGS 0x07DA  // page 200,201
#define APT_MGMSG_PZ_REQ_TSG_IOSETTINGS 0x07DB  // page 200,201
#define APT_MGMSG_PZ_GET_TSG_IOSETTINGS 0x07DC  // page 200,201
#define APT_MGMSG_PZ_REQ_TSG_READING 0x07DD // page 202
#define APT_MGMSG_PZ_GET_TSG_READING 0x07DE // page 202
#define APT_MGMSG_KSG_SET_KCUBEMMIPARAMS 0x07F6 // page 203,204
#define APT_MGMSG_KSG_REQ_KCUBEMMIPARAMS 0x07F7 // page 203,204
#define APT_MGMSG_KSG_GET_KCUBEMMIPARAMS 0x07F8 // page 203,204
#define APT_MGMSG_KSG_SET_KCUBETRIGIOCONFIG 0x07F9 // page 205,206,207
#define APT_MGMSG_KSG_REQ_KCUBETRIGIOCONFIG 0x07FA // page 205,206,207
#define APT_MGMSG_KSG_GET_KCUBETRIGIOCONFIG 0x07FB // page 205,206,207

// NanoTrak Control Messages // page 208
#define APT_MGMSG_PZ_SET_NTMODE 0x0603 // page 209
#define APT_MGMSG_PZ_REQ_NTMODE 0x0604 // page 210
#define APT_MGMSG_PZ_GET_NTMODE 0x0605 // page 210
#define APT_MGMSG_PZ_SET_NTTRACKTHRESHOLD 0x0606 // page 211
#define APT_MGMSG_PZ_REQ_NTTRACKTHRESHOLD 0x0607 // page 211
#define APT_MGMSG_PZ_GET_NTTRACKTHRESHOLD 0x0608 // page 211
#define APT_MGMSG_PZ_SET_NTCIRCHOMEPOS 0x0609 // page 212
#define APT_MGMSG_PZ_REQ_NTCIRCHOMEPOS 0x0610 // page 212
#define APT_MGMSG_PZ_GET_NTCIRCHOMEPOS 0x0611 // page 212
#define APT_MGMSG_PZ_MOVE_NTCIRCTOHOMEPOS 0x0612 // page 213
#define APT_MGMSG_PZ_REQ_NTCIRCCENTREPOS 0x0613 // page 214,215
#define APT_MGMSG_PZ_GET_NTCIRCCENTREPOS 0x0614 // page 214,215
#define APT_MGMSG_PZ_SET_NTCIRCPARAMS 0x0618 // page 216,217,218
#define APT_MGMSG_PZ_REQ_NTCIRCPARAMS 0x0619 // page 216,217,218
#define APT_MGMSG_PZ_GET_NTCIRCPARAMS 0x0620 // page 216,217,218
#define APT_MGMSG_PZ_SET_NTCIRCDIA 0x061A // page 219
#define APT_MGMSG_PZ_SET_NTCIRCDIALUT 0x0621 // page 220,221
#define APT_MGMSG_PZ_REQ_NTCIRCDIALUT 0x0622 // page 220,221
#define APT_MGMSG_PZ_GET_NTCIRCDIALUT 0x0623 // page 220,221
#define APT_MGMSG_PZ_SET_NTPHASECOMPPARAMS 0x0626 // page 222,223
#define APT_MGMSG_PZ_REQ_NTPHASECOMPPARAMS 0x0627 // page 222,223
#define APT_MGMSG_PZ_GET_NTPHASECOMPPARAMS 0x0628 // page 222,223
#define APT_MGMSG_PZ_SET_NTTIARANGEPARAMS 0x0630 // page 224,225,226
#define APT_MGMSG_PZ_REQ_NTTIARANGEPARAMS 0x0631 // page 224,225,226
#define APT_MGMSG_PZ_GET_NTTIARANGEPARAMS 0x0632 // page 224,225,226
#define APT_MGMSG_PZ_SET_NTGAINPARAMS 0x0633 // page 227
#define APT_MGMSG_PZ_REQ_NTGAINPARAMS 0x0634 // page 227
#define APT_MGMSG_PZ_GET_NTGAINPARAMS 0x0635 // page 227
#define APT_MGMSG_PZ_SET_NTTIALPFILTERPARAMS 0x0636 // page 228,229
#define APT_MGMSG_PZ_REQ_NTTIALPFILTERPARAMS 0x0637 // page 228,229
#define APT_MGMSG_PZ_GET_NTTIALPFILTERPARAMS 0x0638 // page 228,229
#define APT_MGMSG_PZ_REQ_NTTIAREADING 0x0639 // page 230,231
#define APT_MGMSG_PZ_GET_NTTIAREADING 0x063A // page 230,231
#define APT_MGMSG_PZ_SET_NTFEEDBACKSRC 0x063B // page 232,233
#define APT_MGMSG_PZ_REQ_NTFEEDBACKSRC 0x063C // page 232,233
#define APT_MGMSG_PZ_GET_NTFEEDBACKSRC 0x063D // page 232,233
#define APT_MGMSG_PZ_REQ_NTSTATUSBITS 0x063E  // page 234,235
#define APT_MGMSG_PZ_GET_NTSTATUSBITS 0x063F  // page 234,235
#define APT_MGMSG_PZ_REQ_NTSTATUSUPDATE 0x0664 // page 236,237,238,239
#define APT_MGMSG_PZ_GET_NTSTATUSUPDATE 0x0665 // page 236,237,238,239
#define APT_MGMSG_PZ_ACK_NTSTATUSUPDATE 0x0666 // page 240
#define APT_MGMSG_KNA_SET_NTTIALPFILTERCOEFFS 0x0687 // page 241,242
#define APT_MGMSG_KNA_REQ_NTTIALPFILTERCOEFFS 0x0688 // page 241,242
#define APT_MGMSG_KNA_GET_NTTIALPFILTERCOEFFS 0x0689 // page 241,242
#define APT_MGMSG_KNA_SET_KCUBEMMIPARAMS 0x068A // page 243,244
#define APT_MGMSG_KNA_REQ_KCUBEMMIPARAMS 0x068B // page 243,244
#define APT_MGMSG_KNA_GET_KCUBEMMIPARAMS 0x068C // page 243,244
#define APT_MGMSG_KNA_SET_KCUBETRIGIOCONFIG 0x068D // page 245,246,247
#define APT_MGMSG_KNA_REQ_KCUBETRIGIOCONFIG 0x068E // page 245,246,247
#define APT_MGMSG_KNA_GET_KCUBETRIGIOCONFIG 0x068F // page 245,246,247
#define APT_MGMSG_KNA_REQ_XYSCAN 0x06A0 // page 248,249
#define APT_MGMSG_KNA_GET_XYSCAN 0x06A1 // page 248,249
#define APT_MGMSG_KNA_STOP_XYSCAN 0x06A2 // page 248,249
#define APT_MGMSG_NT_SET_EEPROMPARAMS 0x07E7 // page 250
#define APT_MGMSG_NT_SET_TNA_DISPSETTINGS 0x07E8 // page 251
#define APT_MGMSG_NT_REQ_TNA_DISPSETTINGS 0x07E9 // page 251
#define APT_MGMSG_NT_GET_TNA_DISPSETTINGS 0x07EA // page 251
#define APT_MGMSG_NT_SET_TNAIOSETTINGS 0x07EB // page 252,253,254
#define APT_MGMSG_NT_REQ_TNAIOSETTINGS 0x07EC // page 252,253,254
#define APT_MGMSG_NT_GET_TNAIOSETTINGS 0x07ED // page 252,253,254

// Laser Control Messages // page 255
#define APT_MGMSG_LA_SET_PARAMS 0x0800 // page 256,257,258,259,260.261,262,263,264,265,266,267,268
#define APT_MGMSG_LA_REQ_PARAMS 0x0801 // page 256,257,258,259,260.261,262,263,264,265,266,267,268
#define APT_MGMSG_LA_GET_PARAMS 0x0802 // page 256,257,258,259,260.261,262,263,264,265,266,267,268
#define APT_MGMSG_LA_SET_EEPROMPARAMS 0x0810 // page 269
#define APT_MGMSG_LA_ENABLEOUTPUT 0x0811 // page 270
#define APT_MGMSG_LA_DISABLEOUTPUT 0x0812 // page 270
#define APT_MGMSG_LD_OPENLOOP 0x0813 // page 271
#define APT_MGMSG_LD_CLOSEDLOOP 0x0814 // page 271
#define APT_MGMSG_LD_POTROTATING 0x0815 // page 272
#define APT_MGMSG_LD_MAXCURRENTADJUST 0x0816 // page 273
#define APT_MGMSG_LD_SET_MAXCURRENTDIGPOT 0x0817 // page 274
#define APT_MGMSG_LD_REQ_MAXCURRENTDIGPOT 0x0818 // page 274
#define APT_MGMSG_LD_GET_MAXCURRENTDIGPOT 0x0819 // page 274
#define APT_MGMSG_LD_FINDTIAGAIN 0x081A // page 275
#define APT_MGMSG_LD_TIAGAINADJUST 0x081B // page 276
#define APT_MGMSG_LA_REQ_STATUSUPDATE 0x0820 // page 277
#define APT_MGMSG_LA_GET_STATUSUPDATE 0x0821 // page 277
#define APT_MGMSG_LA_ACK_STATUSUPDATE 0x0822 // page 279
#define APT_MGMSG_LD_REQ_STATUSUPDATE 0x0825 // page 280,281
#define APT_MGMSG_LD_GET_STATUSUPDATE 0x0826 // page 280,281
#define APT_MGMSG_LD_ACK_STATUSUPDATE 0x0827 // page 282
#define APT_MGMSG_LA_SET_KCUBETRIGIOCONFIG 0x082A // page 283,284,285
#define APT_MGMSG_LA_REQ_KCUBETRIGCONFIG 0x082B // page 283,284,285
#define APT_MGMSG_LA_GET_KCUBETRIGCONFIG 0x082C // page 283,284,285

// Quad Control Messages // page 286
#define APT_MGMSG_QUAD_SET_PARAMS 0x0870 // page 287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309
#define APT_MGMSG_QUAD_REQ_PARAMS 0x0871 // page 287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309
#define APT_MGMSG_QUAD_GET_PARAMS 0x0872 // page 287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309
#define APT_MGMSG_QUAD_REQ_STATUSUPDATE 0x0880 // page 310,311
#define APT_MGMSG_QUAD_GET_STATUSUPDATE 0x0881 // page 310,311
#define APT_MGMSG_QUAD_ACK_STATUSUPDATE 0x0882 // page 311
#define APT_MGMSG_QUAD_SET_EEPROMPARAMS 0x0875 // page 312

// TEC Control Messages // page 313
#define APT_MGMSG_TEC_SET_PARAMS 0x0840 // page 314,315,316,317,318,319,320,321,322,323,324
#define APT_MGMSG_TEC_REQ_PARAMS 0x0841 // page 314,315,316,317,318,319,320,321,322,323,324
#define APT_MGMSG_TEC_GET_PARAMS 0x0842 // page 314,315,316,317,318,319,320,321,322,323,324
#define APT_MGMSG_TEC_SET_EEPROMPARAMS 0x0850 // page 325
#define APT_MGMSG_TEC_REQ_STATUSUPDATE 0x0860 // page 326,327
#define APT_MGMSG_TEC_GET_STATUSUPDATE 0x0861 // page 326,327
#define APT_MGMSG_TEC_ACK_STATUSUPDATE 0x0862 // page 327,328

// TIM and KIM Control Messages
#define APT_MGMSG_PZMOT_SET_PARAMS 0x08C0 // page 330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360
#define APT_MGMSG_PZMOT_REQ_PARAMS 0x08C1 // page 330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360
#define APT_MGMSG_PZMOT_GET_PARAMS 0x08C2 // page 330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360
#define APT_MGMSG_PZMOT_MOVE_ABSOLUTE 0x08D4 // page 361
#define APT_MGMSG_PZMOT_MOVE_COMPLETED 0x08D6 // page 362
#define APT_MGMSG_PZMOT_MOVE_JOG 0x08D9 // page 363
#define APT_MGMSG_PZMOT_REQ_STATUSUPDATE 0x08E0 // page 364
#define APT_MGMSG_PZMOT_GET_STATUSUPDATE 0x08E1 // page 364
#define APT_MGMSG_PZMOT_ACK_STATUSUPDATE 0x08E2 // page 365

// MPC220 and MPC320 Control Messages // page 377
#define APT_MGMSG_POL_SET_PARAMS 0x0530 // page 367,368
#define APT_MGMSG_POL_REQ_PARAMS 0x0531 // page 367,368
#define APT_MGMSG_POL_GET_PARAMS 0x0532 // page 367,368



#endif /* __APT_DEF_H__ */