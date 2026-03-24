#####################################################
# Version: 1.0.0                                    #
# Last modified date: 25.12.16                      #
# Changes log: AD_RANGED 4~20 added                 #
#####################################################


# -*- coding: utf-8 -*-

import ctypes


# ------------------------------------------------------------------
#                 Device Type Defines.
# ------------------------------------------------------------------
DEVTYPE_EZI_SERVO_PLUS_R_ST = 1
DEVTYPE_EZI_SERVO_PLUS_R_ADC_ST = 3
DEVTYPE_EZI_MOTIONLINK = 10
DEVTYPE_EZI_MOTIONLINK2 = 11
DEVTYPE_EZI_STEP_PLUS_R_ST = 20
DEVTYPE_EZI_SERVO_PLUS_R_ALL_ST = 30
DEVTYPE_EZI_SERVO_PLUS_R_ALL_ABS = 35
DEVTYPE_EZI_STEP_PLUS_R_ALL_ST = 40
DEVTYPE_EZI_SERVO_PLUS_R_MINI = 50
DEVTYPE_EZI_STEP_PLUS_R_MINI = 60
DEVTYPE_EZI_SERVO_PLUS_R_ALL60I = 31
DEVTYPE_EZI_SERVO2_PLUS_R_ST = 100
DEVTYPE_EZI_SERVO2_PLUS_R_TO = 180
DEVTYPE_S_SERVO_PLUS_R_ST = 102
DEVTYPE_EZI_SERVO2_PLUS_E_ST = 100
DEVTYPE_EZI_SERVO2_PLUS_E_ST_86 = 101
DEVTYPE_EZI_SERVO2_PLUS_E_ALL60 = 90
DEVTYPE_S_SERVO_PLUS_E_ST = 102
DEVTYPE_EZI_SERVO_PLUS_R_ALL_28 = 105
DEVTYPE_EZI_SERVO_PLUS_R_ALL_28_V2 = 108
DEVTYPE_EZI_SERVO2_CC_LINK_ST = 120
DEVTYPE_EZI_STEP2_PLUS_R_ST = 130
DEVTYPE_EZI_STEP2_PLUS_E_ST = 130

device_names = {
    DEVTYPE_EZI_SERVO_PLUS_R_ST: "Ezi-SERVO Plus-R-ST",
    DEVTYPE_EZI_SERVO_PLUS_R_ADC_ST: "Ezi-SERVO Plus-R-ADC",
    DEVTYPE_EZI_MOTIONLINK: "Ezi-MotionLink",
    DEVTYPE_EZI_MOTIONLINK2: "Ezi-MotionLink2",
    DEVTYPE_EZI_STEP_PLUS_R_ST: "Ezi-STEP Plus-R-ST",
    DEVTYPE_EZI_SERVO_PLUS_R_ALL_ST: "Ezi-SERVO Plus-R-ALL",
    DEVTYPE_EZI_SERVO_PLUS_R_ALL_ABS: "Ezi-SERVO Plus-R-ALL Abs",
    DEVTYPE_EZI_STEP_PLUS_R_ALL_ST: "Ezi-STEP Plus-R-ALL",
    DEVTYPE_EZI_SERVO_PLUS_R_MINI: "Ezi-SERVO Plus-R-Mini",
    DEVTYPE_EZI_STEP_PLUS_R_MINI: "Ezi-STEP Plus-R-Mini",
    DEVTYPE_EZI_SERVO_PLUS_R_ALL60I: "Ezi-SERVO Plus-R-ALL 60i",
    DEVTYPE_EZI_SERVO2_PLUS_R_ST: "Ezi-SERVO II Plus-R-ST",
    DEVTYPE_EZI_SERVO2_PLUS_R_TO: "Ezi-SERVO II Plus-R TO",
    DEVTYPE_S_SERVO_PLUS_R_ST: "S-SERVO Plus-R-ST",
    DEVTYPE_EZI_SERVO2_PLUS_E_ST: "Ezi-SERVO II Plus-E-ST",
    DEVTYPE_EZI_SERVO2_PLUS_E_ST_86: "Ezi-SERVO II Plus-E-ST-86",
    DEVTYPE_EZI_SERVO2_PLUS_E_ALL60: "Ezi-SERVO II Plus-E-ALL60",
    DEVTYPE_S_SERVO_PLUS_E_ST: "S-SERVO Plus-E-ST",
    DEVTYPE_EZI_SERVO_PLUS_R_ALL_28: "Ezi-SERVO Plus-R ALL-28",
    DEVTYPE_EZI_SERVO_PLUS_R_ALL_28_V2: "Ezi-SERVO Plus-R ALL-28 V2",
    DEVTYPE_EZI_SERVO2_CC_LINK_ST: "Ezi-SERVO II CC-LINK-ST",
    DEVTYPE_EZI_STEP2_PLUS_R_ST: "Ezi-STEP II Plus-R-ST",
    DEVTYPE_EZI_STEP2_PLUS_E_ST: "Ezi-STEP II Plus-E-ST",
}

# ------------------------------------------------------------------
#                 Device Type Defines. (for V8)
# ------------------------------------------------------------------

DEVTYPEV8_EZI_SERVO_PLUS_R_ST = 1
DEVTYPEV8_EZI_MOTIONLINK = 10
DEVTYPEV8_EZI_STEP_PLUS_R_ST = 20
DEVTYPEV8_EZI_SERVO_PLUS_R_ABS_ST = 30
DEVTYPEV8_EZI_SERVO_PLUS_R_BLDC_ST = 40
DEVTYPEV8_EZI_MOTIONLINK2 = 110

device_names_v8 = {
    DEVTYPEV8_EZI_SERVO_PLUS_R_ST: "Ezi-SERVO Plus-R Inc.",
    DEVTYPEV8_EZI_MOTIONLINK: "Ezi-MotionLink",
    DEVTYPEV8_EZI_STEP_PLUS_R_ST: "Ezi-STEP Plus-R",
    DEVTYPEV8_EZI_SERVO_PLUS_R_ABS_ST: "Ezi-SERVO Plus-R Abs.",
    DEVTYPEV8_EZI_SERVO_PLUS_R_BLDC_ST: "Ezi-SERVO Plus-R BLDC F60",
    DEVTYPEV8_EZI_MOTIONLINK2: "Ezi-MotionLink2",
}

# ------------------------------------------------------------------
#                 Device Type Defines. (Ezi-IO)
# ------------------------------------------------------------------
DEVTYPE_EZI_IO_PLUS_R_IN16 = 150
DEVTYPE_EZI_IO_PLUS_R_OUT16 = 160
DEVTYPE_EZI_IO_PLUS_R_I8O8 = 155
DEVTYPE_EZI_IO_PLUS_E_IN16 = 150
DEVTYPE_EZI_IO_PLUS_E_IN32 = 151
DEVTYPE_EZI_IO_PLUS_E_OUT16 = 160
DEVTYPE_EZI_IO_PLUS_E_OUT32 = 161
DEVTYPE_EZI_IO_PLUS_E_I8O8 = 155
DEVTYPE_EZI_IO_PLUS_E_I16O16 = 156


device_names_ezi_io = {
    DEVTYPE_EZI_IO_PLUS_R_IN16: "Ezi-IO RS-485-IN16",
    DEVTYPE_EZI_IO_PLUS_R_OUT16: "Ezi-IO RS-485-OUT16",
    DEVTYPE_EZI_IO_PLUS_R_I8O8: "Ezi-IO RS-485-I8O8",
    DEVTYPE_EZI_IO_PLUS_E_IN16: "Ezi-IO Ethernet-IN16",
    DEVTYPE_EZI_IO_PLUS_E_IN32: "Ezi-IO Ethernet-IN32",
    DEVTYPE_EZI_IO_PLUS_E_OUT16: "Ezi-IO Ethernet-OUT16",
    DEVTYPE_EZI_IO_PLUS_E_OUT32: "Ezi-IO Ethernet-OUT32",
    DEVTYPE_EZI_IO_PLUS_E_I8O8: "Ezi-IO Ethernet-I8O8",
    DEVTYPE_EZI_IO_PLUS_E_I16O16: "Ezi-IO Ethernet-I16O16",
}

# ------------------------------------------------------------------
#                 Device Type Defines. (Etc)
# ------------------------------------------------------------------
DEVTYPE_BOOT_ROM = 0xFF
DEVTYPE_BOOT_ROM_2 = 0xFE

# ------------------------------------------------------------------
#                 Motion Direction Defines.
# ------------------------------------------------------------------
DIR_INC = 1
DIR_DEC = 0

# ------------------------------------------------------------------
#                 Axis Status Flag Defines.
# ------------------------------------------------------------------
MAX_AXIS_STATUS = 32

# ------------------------------------------------------------------
#                 GetAllStatusEx Function
# ------------------------------------------------------------------
ALLSTATUSEX_ITEM_COUNT = 12

STATUSEX_TYPE_NONE = 0
STATUSEX_TYPE_INPUT = 1
STATUSEX_TYPE_OUTPUT = 2
STATUSEX_TYPE_AXISSTATUS = 3
STATUSEX_TYPE_CMDPOS = 4
STATUSEX_TYPE_ACTPOS = 5
STATUSEX_TYPE_ACTVEL = 6
STATUSEX_TYPE_POSERR = 7
STATUSEX_TYPE_PTNO = 8
STATUSEX_TYPE_ALARMTYPE = 9
STATUSEX_TYPE_TEMPERATURE = 10
STATUSEX_TYPE_CURRENT = 11
STATUSEX_TYPE_LOAD = 12
STATUSEX_TYPE_PEAKLOAD = 13
STATUSEX_TYPE_ENCVEL = 14
STATUSEX_TYPE_INPUT_HIGH = 15
STATUSEX_TYPE_PTNO_RUNNING = 16
STATUSEX_TYPE_ADVALUE0 = 30  # Ezi-IO AD: Ch0 & Ch1 AD Value
STATUSEX_TYPE_ADVALUE2 = 31  # Ezi-IO AD: Ch2 & Ch3 AD Value
STATUSEX_TYPE_ADVALUE4 = 32  # Ezi-IO AD: Ch4 & Ch5 AD Value
STATUSEX_TYPE_ADVALUE6 = 33  # Ezi-IO AD: Ch6 & Ch7 AD Value
STATUSEX_TYPE_CNTSTATUS = 40  # Ezi-IO COUNTER: Status
STATUSEX_TYPE_CNTCH1_COUNT = 41  # Ezi-IO COUNTER: CH1 COUNT
STATUSEX_TYPE_CNTCH1_LATCHA = 42  # Ezi-IO COUNTER: CH1 LATCHA
STATUSEX_TYPE_CNTCH1_LATCHB = 43  # Ezi-IO COUNTER: CH1 LATCHB
STATUSEX_TYPE_CNTCH1_ZLATCH = 44  # Ezi-IO COUNTER: CH1 Z LATCH
STATUSEX_TYPE_CNTCH2_COUNT = 45  # Ezi-IO COUNTER: CH2 COUNT
STATUSEX_TYPE_CNTCH2_LATCHA = 46  # Ezi-IO COUNTER: CH2 LATCHA
STATUSEX_TYPE_CNTCH2_LATCHB = 47  # Ezi-IO COUNTER: CH2 LATCHB
STATUSEX_TYPE_CNTCH2_ZLATCH = 48  # Ezi-IO COUNTER: CH2 Z LATCH

status_names = {
    STATUSEX_TYPE_NONE: "None",
    STATUSEX_TYPE_INPUT: "Input",
    STATUSEX_TYPE_OUTPUT: "Output",
    STATUSEX_TYPE_AXISSTATUS: "Axis Status",
    STATUSEX_TYPE_CMDPOS: "Command Position",
    STATUSEX_TYPE_ACTPOS: "Actual Position",
    STATUSEX_TYPE_ACTVEL: "Actual Velocity",
    STATUSEX_TYPE_POSERR: "Position Error",
    STATUSEX_TYPE_PTNO: "Point Number",
    STATUSEX_TYPE_ALARMTYPE: "Alarm Type",
    STATUSEX_TYPE_TEMPERATURE: "Temperature",
    STATUSEX_TYPE_CURRENT: "Current",
    STATUSEX_TYPE_LOAD: "Load",
    STATUSEX_TYPE_PEAKLOAD: "Peak Load",
    STATUSEX_TYPE_ENCVEL: "Encoder Velocity",
    STATUSEX_TYPE_INPUT_HIGH: "Input High",
    STATUSEX_TYPE_PTNO_RUNNING: "Point Number Running",
    STATUSEX_TYPE_ADVALUE0: "AD Value 0",
    STATUSEX_TYPE_ADVALUE2: "AD Value 2",
    STATUSEX_TYPE_ADVALUE4: "AD Value 4",
    STATUSEX_TYPE_ADVALUE6: "AD Value 6",
    STATUSEX_TYPE_CNTSTATUS: "Counter Status",
    STATUSEX_TYPE_CNTCH1_COUNT: "CH1 Count",
    STATUSEX_TYPE_CNTCH1_LATCHA: "CH1 Latch A",
    STATUSEX_TYPE_CNTCH1_LATCHB: "CH1 Latch B",
    STATUSEX_TYPE_CNTCH1_ZLATCH: "CH1 Z Latch",
    STATUSEX_TYPE_CNTCH2_COUNT: "CH2 Count",
    STATUSEX_TYPE_CNTCH2_LATCHA: "CH2 Latch A",
    STATUSEX_TYPE_CNTCH2_LATCHB: "CH2 Latch B",
    STATUSEX_TYPE_CNTCH2_ZLATCH: "CH2 Z Latch",
}

# ------------------------------------------------------------------
#                 Input/Output Assigning Defines.
# ------------------------------------------------------------------

LEVEL_LOW_ACTIVE = 0
LEVEL_HIGH_ACTIVE = 1

IN_LOGIC_NONE = 0
OUT_LOGIC_NONE = 0

# ------------------------------------------------------------------
#                 POSITION TABLE Defines.
# ------------------------------------------------------------------

MAX_LOOP_COUNT = 100
MAX_WAIT_TIME = 60000

PUSH_RATIO_MIN = 20
PUSH_RATIO_MAX = 90

PUSH_SPEED_MIN = 1
PUSH_SPEED_MAX = 100000

PUSH_PULSECOUNT_MIN = 1
PUSH_PULSECOUNT_MAX = 10000

# COMMAND_LIST
CMD_ABS_LOWSPEED = 0
CMD_ABS_HIGHSPEED = 1
CMD_ABS_HIGHSPEEDDECEL = 2
CMD_ABS_NORMALMOTION = 3
CMD_INC_LOWSPEED = 4
CMD_INC_HIGHSPEED = 5
CMD_INC_HIGHSPEEDDECEL = 6
CMD_INC_NORMALMOTION = 7
CMD_MOVE_ORIGIN = 8
CMD_COUNTERCLEAR = 9
CMD_PUSH_ABSMOTION = 10
CMD_STOP = 11

CMD_MAX_COUNT = 12

CMD_NO_COMMAND = 0xFFFF


class ITEM_NODE(object):
    def __init__(
        self,
        lPosition=0,
        dwStartSpd=0,
        dwMoveSpd=0,
        wAccelRate=0,
        wDecelRate=0,
        wCommand=0,
        wWaitTime=0,
        wContinuous=0,
        wBranch=0,
        wCond_branch0=0,
        wCond_branch1=0,
        wCond_branch2=0,
        wLoopCount=0,
        wBranchAfterLoop=0,
        wPTSet=0,
        wLoopCountCLR=0,
        bCheckInpos=0,
        lTriggerPos=0,
        wTriggerOnTime=0,
        wPushRatio=0,
        dwPushSpeed=0,
        lPushPosition=0,
        wPushMode=0,
    ):
        self.lPosition = lPosition
        self.dwStartSpd = dwStartSpd
        self.dwMoveSpd = dwMoveSpd
        self.wAccelRate = wAccelRate
        self.wDecelRate = wDecelRate
        self.wCommand = wCommand
        self.wWaitTime = wWaitTime
        self.wContinuous = wContinuous
        self.wBranch = wBranch
        self.wCond_branch0 = wCond_branch0
        self.wCond_branch1 = wCond_branch1
        self.wCond_branch2 = wCond_branch2
        self.wLoopCount = wLoopCount
        self.wBranchAfterLoop = wBranchAfterLoop
        self.wPTSet = wPTSet
        self.wLoopCountCLR = wLoopCountCLR
        self.bCheckInpos = bCheckInpos  # 0 : Check Inpos, 1 : Don't Check.
        self.lTriggerPos = lTriggerPos
        self.wTriggerOnTime = wTriggerOnTime
        self.wPushRatio = wPushRatio
        self.dwPushSpeed = dwPushSpeed
        self.lPushPosition = lPushPosition
        self.wPushMode = wPushMode

    def __repr__(self):
        return (
            "ITEM_NODE(lPosition=%d, dwStartSpd=%d, dwMoveSpd=%d, wAccelRate=%d, "
            "wDecelRate=%d, wCommand=%d, wWaitTime=%d, wContinuous=%d, wBranch=%d, "
            "wCond_branch0=%d, wCond_branch1=%d, wCond_branch2=%d, wLoopCount=%d, "
            "wBranchAfterLoop=%d, wPTSet=%d, wLoopCountCLR=%d, bCheckInpos=%d, "
            "lTriggerPos=%d, wTriggerOnTime=%d, wPushRatio=%d, dwPushSpeed=%d, "
            "lPushPosition=%d, wPushMode=%d)" % (
                self.lPosition, self.dwStartSpd, self.dwMoveSpd, self.wAccelRate,
                self.wDecelRate, self.wCommand, self.wWaitTime, self.wContinuous,
                self.wBranch, self.wCond_branch0, self.wCond_branch1, self.wCond_branch2,
                self.wLoopCount, self.wBranchAfterLoop, self.wPTSet, self.wLoopCountCLR,
                self.bCheckInpos, self.lTriggerPos, self.wTriggerOnTime, self.wPushRatio,
                self.dwPushSpeed, self.lPushPosition, self.wPushMode
            )
        )




OFFSET_POSITION = 0
OFFSET_LOWSPEED = 4
OFFSET_HIGHSPEED = 8
OFFSET_ACCELTIME = 12
OFFSET_DECELTIME = 14
OFFSET_COMMAND = 16
OFFSET_WAITTIME = 18
OFFSET_CONTINUOUS = 20
OFFSET_JUMPTABLENO = 22
OFFSET_JUMPPT0 = 24
OFFSET_JUMPPT1 = 26
OFFSET_JUMPPT2 = 28
OFFSET_LOOPCOUNT = 30
OFFSET_LOOPJUMPTABLENO = 32
OFFSET_PTSET = 34
OFFSET_LOOPCOUNTCLEAR = 36
OFFSET_CHECKINPOSITION = 38
OFFSET_TRIGGERPOSITION = 40
OFFSET_TRIGGERONTIME = 44
OFFSET_PUSHRATIO = 46
OFFSET_PUSHSPEED = 48
OFFSET_PUSHPOSITION = 52
OFFSET_PUSHMODE = 56

OFFSET_BLANK = 58

# ------------------------------------------------------------------
#                 EX Commands Option Defines.
# ------------------------------------------------------------------


class MOTION_OPTION_EX(object):
    def __init__(
        self,
        BIT_IGNOREEXSTOP=0,
        BIT_USE_CUSTOMACCEL=0,
        BIT_USE_CUSTOMDECEL=0,
        wCustomAccelTime=0,
        wCustomDecelTime=0,
        buffReserved=None
    ):
        self.BIT_IGNOREEXSTOP = BIT_IGNOREEXSTOP
        self.BIT_USE_CUSTOMACCEL = BIT_USE_CUSTOMACCEL
        self.BIT_USE_CUSTOMDECEL = BIT_USE_CUSTOMDECEL
        self.wCustomAccelTime = wCustomAccelTime
        self.wCustomDecelTime = wCustomDecelTime
        self.buffReserved = buffReserved if buffReserved is not None else [0] * 24

    def __repr__(self):
        return (
            "MOTION_OPTION_EX(BIT_IGNOREEXSTOP=%d, "
            "BIT_USE_CUSTOMACCEL=%d, "
            "BIT_USE_CUSTOMDECEL=%d, "
            "wCustomAccelTime=%d, "
            "wCustomDecelTime=%d)" % (
                self.BIT_IGNOREEXSTOP,
                self.BIT_USE_CUSTOMACCEL,
                self.BIT_USE_CUSTOMDECEL,
                self.wCustomAccelTime,
                self.wCustomDecelTime
            )
        )




class VELOCITY_OPTION_EX(object):
    def __init__(
        self,
        BIT_IGNOREEXSTOP=0,
        BIT_USE_CUSTOMACCDEC=0,
        wCustomAccDecTime=0,
        buffReserved=None
    ):
        self.BIT_IGNOREEXSTOP = BIT_IGNOREEXSTOP
        self.BIT_USE_CUSTOMACCDEC = BIT_USE_CUSTOMACCDEC
        self.wCustomAccDecTime = wCustomAccDecTime
        self.buffReserved = buffReserved if buffReserved is not None else [0] * 26

    def __repr__(self):
        return (
            "VELOCITY_OPTION_EX(BIT_IGNOREEXSTOP=%d, "
            "BIT_USE_CUSTOMACCDEC=%d, "
            "wCustomAccDecTime=%d)" % (
                self.BIT_IGNOREEXSTOP,
                self.BIT_USE_CUSTOMACCDEC,
                self.wCustomAccDecTime
            )
        )




# ------------------------------------------------------------------
#                  Alarm Type Defines.
# ------------------------------------------------------------------
# ALARM_TYPE
ALARM_NONE = 0

ALARM_OVERCURRENT = 1
ALARM_OVERSPEED = 2
ALARM_STEPOUT = 3
ALARM_OVERLOAD = 4
ALARM_OVERTEMPERATURE = 5
ALARM_OVERBACKEMF = 6
ALARM_MOTORCONNECT = 7
ALARM_ENCODERCONNECT = 8
ALARM_LOWMOTORPOWER = 9
ALARM_INPOSITION = 10
ALARM_SYSTEMHALT = 11
ALARM_ROMDEVICE = 12
ALARM_RESERVED0 = 13
ALARM_HIGHINPUTVOLTAGE = 14
ALARM_POSITIONOVERFLOW = 15
ALARM_POSITIONCHANGED = 16

MAX_ALARM_NUM = 17


class ALARM_LOG(object):
    def __init__(self, nAlarmCount=0, nAlarmLog=None):
        if nAlarmLog is None:
            nAlarmLog = [0]
        self.nAlarmCount = nAlarmCount
        self.nAlarmLog = nAlarmLog

    def __repr__(self):
        return "ALARM_LOG(nAlarmCount=%d, nAlarmLog=%s)" % (
            self.nAlarmCount,
            self.nAlarmLog
        )



# ------------------------------------------------------------------
#                   Drive Information Defines.
# ------------------------------------------------------------------
class Drive_Info(object):
    def __init__(
        self,
        nVersionNo=None,  # Drive Version Number (Major Ver/Minor Ver/Bug fix/Build No.) 
        sVersion="",  # Drive Version string
        nDriveType=0,  # Drive Model
        nMotorType=0,  # Motor Model
        sMotorInfo="",  # Motor Info.
        nInPinNo=0,  # Input Pin Number
        nOutPinNo=0,  # Output Pin Number
        nPTNum=0,  # Position Table Item Number
        nFirmwareType=0,  # Firmware Type Information
    ):
        self.nVersionNo = nVersionNo if nVersionNo is not None else [0, 0, 0, 0]
        self.sVersion = sVersion
        self.nDriveType = nDriveType
        self.nMotorType = nMotorType
        self.sMotorInfo = sMotorInfo
        self.nInPinNo = nInPinNo
        self.nOutPinNo = nOutPinNo
        self.nPTNum = nPTNum
        self.nFirmwareType = nFirmwareType

    def __repr__(self):
        return (
            "Drive_Info(nVersionNo=%s, sVersion='%s', nDriveType=%d, nMotorType=%d, "
            "sMotorInfo='%s', nInPinNo=%d, nOutPinNo=%d, nPTNum=%d, nFirmwareType=%d)" % (
                self.nVersionNo, self.sVersion, self.nDriveType, self.nMotorType,
                self.sMotorInfo, self.nInPinNo, self.nOutPinNo, self.nPTNum, self.nFirmwareType
            )
        )


# ------------------------------------------------------------------
#                  I/O Module Defines.
# ------------------------------------------------------------------


class TRIGGER_INFO(object):
    def __init__(self, wPeriod=0, wReserved1=0, wOnTime=0, wReserved2=0, wCount=0):
        self.wPeriod = wPeriod
        self.wReserved1 = wReserved1
        self.wOnTime = wOnTime
        self.wReserved2 = wReserved2
        self.wCount = wCount


# ------------------------------------------------------------------
#                 Ezi-IO AD Defines
# ------------------------------------------------------------------
# AD_RANGE
ADRANGE_10_to_10 = 0  #  -10V ~  10V [2.441mV]
ADRANGE_5_to_5 = 1  #   -5V ~   5V [1.22mV]
ADRANGE_2_5_to_2_5 = 2  # -2.5V ~ 2.5V [0.61mV]
ADRANGE_0_to_10 = 3  #    0V ~  10V [1.22mV]
ADRANGE_0_to_20_C = 4  #   0mA ~ 20mA [2.44uA]
ADRANGE_4_to_20_C = 5  #   4mA ~ 20mA [1.95uA]

ADRANGE_EX_10_to_10 = 0x10  # (Read Only, External Switch)  -10V ~  10V [2.441mV]
ADRANGE_EX_5_to_5 = 17  # (Read Only, External Switch)   -5V ~   5V [1.22mV]
ADRANGE_EX_2_5_to_2_5 = 18  # (Read Only, External Switch) -2.5V ~ 2.5V [0.61mV]
ADRANGE_EX_0_to_10 = 19  # (Read Only, External Switch)    0V ~  10V [1.22mV]


class DataElement(ctypes.Structure):
    _fields_ = [
        ("range", ctypes.c_char),
        ("rawdata", ctypes.c_short),
        ("converted", ctypes.c_float),
    ]


class AD_RESULT(ctypes.Structure):
    _fields_ = [("elements", DataElement * 16)]


# AD_DATA_TYPE
TYPE_AD_RANGE = 0
TYPE_AD_FILTER_LENGTH = 1
TYPE_AD_FILTER_OFFSET = 2


# ------------------------------------------------------------------
#                 Ezi-IO DA Defines
# ------------------------------------------------------------------
# DA_RANGE
DARANGE_0_to_5 = 0  #    0V ~   5V (0 ~ 25000)
DARANGE_10_to_10 = 1  #  -10V ~  10V (-25000 ~ 25000)
DARANGE_0_to_10 = 2  #    0V ~  10V (0 ~ 25000)
DARANGE_1_to_5 = 3  #    1V ~   5V (0 ~ 25000)
DARANGE_0_to_20_C = 4  #   0mA ~ 20mA (0 ~ 25000)
DARANGE_4_to_20_C = 5  #   4mA ~ 20mA (0 ~ 25000)

DARANGE_EX_0_to_5 = 16  # (Read Only, External Switch)    0V ~   5V (0 ~ 25000)
DARANGE_EX_10_to_10 = 17  # (Read Only, External Switch)  -10V ~  10V (-25000 ~ 25000)
DARANGE_EX_0_to_20_C = 18  # (Read Only, External Switch)   0mA ~ 20mA (0 ~ 25000)
DARANGE_EX_4_to_20_C = 19  # (Read Only, External Switch)   4mA ~ 20mA (0 ~ 25000)

# DAC_CONFIG
DAC_RANGE = 0

DAC_CALIBRATION_HIGH = 1
DAC_CALIBRATION_LOW = 2

# ------------------------------------------------------------------
#                 Ezi-IO COUNTER Defines
# ------------------------------------------------------------------
# COUNTER_CMD
CNTCMD_CH_ENABLE = 0
CNTCMD_LATCHA_ENABLE = 1
CNTCMD_LATCHB_ENABLE = 2
CNTCMD_ZLATCH_ENABLE = 3

CNTCMD_RESET_ALL = 4
CNTCMD_RESET_COUNT = 5
CNTCMD_RESET_LATCH = 6

# COUNTER_VALUE
CNT_COUNT_VALUE = 0
CNT_LATCHA_VALUE = 1
CNT_LATCHB_VALUE = 2
CNT_ZLATCH_VALUE = 3


CNTSTATUS_CH1_ENABLED = 0x00000001
CNTSTATUS_CH1_LTCA_EN = 0x00000002
CNTSTATUS_CH1_LTCB_EN = 0x00000004
CNTSTATUS_CH1_ZLTC_EN = 0x00000008
CNTSTATUS_CH1_LTCA_LATCHED = 0x00000010
CNTSTATUS_CH1_LTCB_LATCHED = 0x00000020
CNTSTATUS_CH1_ZLTC_LATCHED = 0x00000040
CNTSTATUS_CH1_RESET = 0x00000080
CNTSTATUS_CH1_TRIGGER = 0x00000100
CNTSTATUS_CH1_COMPARISON = 0x00000200
CNTSTATUS_CH2_ENABLED = 0x00010000
CNTSTATUS_CH2_LTCA_EN = 0x00020000
CNTSTATUS_CH2_LTCB_EN = 0x00040000
CNTSTATUS_CH2_ZLTC_EN = 0x00080000
CNTSTATUS_CH2_LTCA_LATCHED = 0x00100000
CNTSTATUS_CH2_LTCB_LATCHED = 0x00200000
CNTSTATUS_CH2_ZLTC_LATCHED = 0x00400000
CNTSTATUS_CH2_RESET = 0x00800000
CNTSTATUS_CH2_TRIGGER = 0x01000000
CNTSTATUS_CH2_COMPARISON = 0x02000000


# COUNTER_CONFIG
CFG_INPUT_MODE = 0  # Counter Input Mode (0: Quadrature, 1: 2-Pulse, 2: 1-Pulse)
CFG_COUNT_DIRECTION = 1  # Count Direction (0: CW, 1: CCW)
CFG_LATCHA_MODE = 2  # Latch A Mode (0: Single, 1: Continue)
CFG_LATCHB_MODE = 3  # Latch B Mode (0: Single, 1: Continue)
CFG_ZLATCH_MODE = 4  # Z-Phase Latch Mode (0: Single, 1: Continue, 2: Reset)
CFG_LATCHA_LOGIC = 5  # Latch A Logic (0: Low-Active, 1: High-Active)
CFG_LATCHB_LOGIC = 6  # Latch B Logic (0: Low-Active, 1: High-Active)
CFG_ZLATCH_LOGIC = 7  # Z-Phase Latch Logic (0: Low-Active, 1: High-Active)
CFG_RESET_LOGIC = 8  # (external input) Reset Logic (0: Low-Active, 1: High-Active)
CFG_CP_LOGIC = (
    9  # (external output) Comparison Output Logic (0: Low-Active, 1: High-Active)
)

# ------------------------------------------------------------------
#                 LOG Level Defines
# ------------------------------------------------------------------
# LOG_LEVEL
LOG_LEVEL_COMM = 0  # Communication Log only
LOG_LEVEL_PARAM = 1  # Communication Log and parameter functions
LOG_LEVEL_MOTION = 2  # Communication Log and parameter, motion, I/O functions
LOG_LEVEL_ALL = 3  # Communication Log and all functions
