#####################################################
# Version: 1.0.0                                    #
# Last modified date: 25.10.31                      #
# Changes log: First released version               #
#####################################################


# -*- coding: utf-8 -*-

import os
import ctypes
import sys
dll_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'EziMOTIONPlusE.dll'))


EziMOTIONPlusE_dll = ctypes.windll.LoadLibrary(dll_path)

from ctypes import *
from MOTION_DEFINE import *

if sys.version_info[0] < 3:
    c_wchar_p_type = ctypes.c_wchar_p
    def to_unicode(s):
        if isinstance(s, str):
            return unicode(s, "utf-8")
        return s
else:
    c_wchar_p_type = ctypes.c_wchar_p
    def to_unicode(s):
        return s 


# ctypes 구조체 정의
class CDrive_Info(Structure):
    _pack_ = 1  # 1바이트 경계로 정렬
    _fields_ = [
        ("nVersionNo", c_ushort * 4),  # Drive Version Number
        ("sVersion", c_char * 30),  # Drive Version string
        ("nDriveType", c_ushort),  # Drive Model
        ("nMotorType", c_ushort),  # Motor Model
        ("sMotorInfo", c_char * 20),  # Motor Info
        ("nInPinNo", c_ushort),  # Input Pin Number
        ("nOutPinNo", c_ushort),  # Output Pin Number
        ("nPTNum", c_ushort),  # Position Table Item Number
        ("nFirmwareType", c_ushort)  # Firmware Type Information
    ]


def Drive_Info_to_CDrive_Info(d):
    if sys.version_info[0] < 3:  # Python 2.x
        sVersion_encoded = d.sVersion.encode('utf-8') if isinstance(d.sVersion, unicode) else d.sVersion
        sMotorInfo_encoded = d.sMotorInfo.encode('utf-8') if isinstance(d.sMotorInfo, unicode) else d.sMotorInfo
    else:  # Python 3.x
        sVersion_encoded = d.sVersion.encode('utf-8') if isinstance(d.sVersion, str) else d.sVersion
        sMotorInfo_encoded = d.sMotorInfo.encode('utf-8') if isinstance(d.sMotorInfo, str) else d.sMotorInfo

    return CDrive_Info(
        nVersionNo=(c_ushort * 4)(*d.nVersionNo),
        sVersion=sVersion_encoded,
        nDriveType=d.nDriveType,
        nMotorType=d.nMotorType,
        sMotorInfo=sMotorInfo_encoded,
        nInPinNo=d.nInPinNo,
        nOutPinNo=d.nOutPinNo,
        nPTNum=d.nPTNum,
        nFirmwareType=d.nFirmwareType
    )

def CDrive_Info_to_Drive_Info(c):
    if sys.version_info[0] < 3:  # Python 2.x
        sVersion_decoded = c.sVersion.decode('utf-8') if isinstance(c.sVersion, str) else c.sVersion
        sMotorInfo_decoded = c.sMotorInfo.decode('utf-8') if isinstance(c.sMotorInfo, str) else c.sMotorInfo
    else:  # Python 3.x
        sVersion_decoded = c.sVersion.decode('utf-8') if isinstance(c.sVersion, bytes) else c.sVersion
        sMotorInfo_decoded = c.sMotorInfo.decode('utf-8') if isinstance(c.sMotorInfo, bytes) else c.sMotorInfo

    return Drive_Info(
        nVersionNo=list(c.nVersionNo),
        sVersion=sVersion_decoded,
        nDriveType=c.nDriveType,
        nMotorType=c.nMotorType,
        sMotorInfo=sMotorInfo_decoded,
        nInPinNo=c.nInPinNo,
        nOutPinNo=c.nOutPinNo,
        nPTNum=c.nPTNum,
        nFirmwareType=c.nFirmwareType
    )


# ctypes 구조체 정의
class CMOTION_OPTION_EX(Structure):
    _pack_ = 1  # 1바이트 경계로 정렬
    _fields_ = [
        ("BIT_IGNOREEXSTOP", c_uint,1),
        ("BIT_USE_CUSTOMACCEL", c_uint,1),
        ("BIT_USE_CUSTOMDECEL", c_uint,1),
        ("wCustomAccelTime", c_ushort,16),
        ("wCustomDecelTime", c_ushort,16),
        ("buffReserved", c_ubyte * 24)
    ]


# ctypes 구조체를 클래스로 변환하는 함수
def CMOTION_OPTION_EX_to_MOTION_OPTION_EX(c):
    return MOTION_OPTION_EX(
        BIT_IGNOREEXSTOP=c.BIT_IGNOREEXSTOP,
        BIT_USE_CUSTOMACCEL=c.BIT_USE_CUSTOMACCEL,
        BIT_USE_CUSTOMDECEL=c.BIT_USE_CUSTOMDECEL,
        wCustomAccelTime=c.wCustomAccelTime,
        wCustomDecelTime=c.wCustomDecelTime,
        buffReserved = list(c.buffReserved)
    )


# 클래스를 ctypes 구조체로 변환하는 함수
def MOTION_OPTION_EX_to_CMOTION_OPTION_EX(m):
    return CMOTION_OPTION_EX(
        BIT_IGNOREEXSTOP=m.BIT_IGNOREEXSTOP,
        BIT_USE_CUSTOMACCEL=m.BIT_USE_CUSTOMACCEL,
        BIT_USE_CUSTOMDECEL=m.BIT_USE_CUSTOMDECEL,
        wCustomAccelTime=m.wCustomAccelTime,
        wCustomDecelTime=m.wCustomDecelTime,
        buffReserved = (c_ubyte * 24)(*m.buffReserved)
    )


# ctypes 구조체 정의
class CVELOCITY_OPTION_EX(Structure):
    _pack_ = 1  # 1바이트 경계로 정렬
    _fields_ = [
        ("BIT_IGNOREEXSTOP", c_uint,1),
        ("BIT_USE_CUSTOMACCDEC", c_uint,1),
        ("wCustomAccDecTime", c_ushort,16),
        ("buffReserved", c_ubyte * 26)
    ]


def CVELOCITY_OPTION_EX_to_VELOCITY_OPTION_EX(
    c
):
    return VELOCITY_OPTION_EX(
        BIT_IGNOREEXSTOP=c.BIT_IGNOREEXSTOP,
        BIT_USE_CUSTOMACCDEC=c.BIT_USE_CUSTOMACCDEC,
        wCustomAccDecTime=c.wCustomAccDecTime,
        buffReserved = list(c.buffReserved)

    )


def VELOCITY_OPTION_EX_to_CVELOCITY_OPTION_EX(
    v
):
    return CVELOCITY_OPTION_EX(
        BIT_IGNOREEXSTOP=v.BIT_IGNOREEXSTOP,
        BIT_USE_CUSTOMACCDEC=v.BIT_USE_CUSTOMACCDEC,
        wCustomAccDecTime=v.wCustomAccDecTime,
        buffReserved = (c_ubyte * 26)(*v.buffReserved)
    )


class CITEM_NODE(Structure):
    _pack_ = 2  # 1바이트 경계로 정렬
    _fields_ = [
        ("lPosition", c_int),
        ("dwStartSpd", c_uint),
        ("dwMoveSpd", c_uint),
        ("wAccelRate", c_ushort),
        ("wDecelRate", c_ushort),
        ("wCommand", c_ushort),
        ("wWaitTime", c_ushort),
        ("wContinuous", c_ushort),
        ("wBranch", c_ushort),
        ("wCond_branch0", c_ushort),
        ("wCond_branch1", c_ushort),
        ("wCond_branch2", c_ushort),
        ("wLoopCount", c_ushort),
        ("wBranchAfterLoop", c_ushort),
        ("wPTSet", c_ushort),
        ("wLoopCountCLR", c_ushort),
        ("bCheckInpos", c_ushort),
        ("lTriggerPos", c_int),
        ("wTriggerOnTime", c_ushort),
        ("wPushRatio", c_ushort),
        ("dwPushSpeed", c_uint),
        ("lPushPosition", c_int),
        ("wPushMode", c_ushort)
    ]


def CITEM_NODE_to_ITEM_NODE(c):
    return ITEM_NODE(
        lPosition=c.lPosition,
        dwStartSpd=c.dwStartSpd,
        dwMoveSpd=c.dwMoveSpd,
        wAccelRate=c.wAccelRate,
        wDecelRate=c.wDecelRate,
        wCommand=c.wCommand,
        wWaitTime=c.wWaitTime,
        wContinuous=c.wContinuous,
        wBranch=c.wBranch,
        wCond_branch0=c.wCond_branch0,
        wCond_branch1=c.wCond_branch1,
        wCond_branch2=c.wCond_branch2,
        wLoopCount=c.wLoopCount,
        wBranchAfterLoop=c.wBranchAfterLoop,
        wPTSet=c.wPTSet,
        wLoopCountCLR=c.wLoopCountCLR,
        bCheckInpos=c.bCheckInpos,
        lTriggerPos=c.lTriggerPos,
        wTriggerOnTime=c.wTriggerOnTime,
        wPushRatio=c.wPushRatio,
        dwPushSpeed=c.dwPushSpeed,
        lPushPosition=c.lPushPosition,
        wPushMode=c.wPushMode
    )


def ITEM_NODE_to_CITEM_NODE(v):
    return CITEM_NODE(
        lPosition=v.lPosition,
        dwStartSpd=v.dwStartSpd,
        dwMoveSpd=v.dwMoveSpd,
        wAccelRate=v.wAccelRate,
        wDecelRate=v.wDecelRate,
        wCommand=v.wCommand,
        wWaitTime=v.wWaitTime,
        wContinuous=v.wContinuous,
        wBranch=v.wBranch,
        wCond_branch0=v.wCond_branch0,
        wCond_branch1=v.wCond_branch1,
        wCond_branch2=v.wCond_branch2,
        wLoopCount=v.wLoopCount,
        wBranchAfterLoop=v.wBranchAfterLoop,
        wPTSet=v.wPTSet,
        wLoopCountCLR=v.wLoopCountCLR,
        bCheckInpos=v.bCheckInpos,
        lTriggerPos=v.lTriggerPos,
        wTriggerOnTime=v.wTriggerOnTime,
        wPushRatio=v.wPushRatio,
        dwPushSpeed=v.dwPushSpeed,
        lPushPosition=v.lPushPosition,
        wPushMode=v.wPushMode
    )


class CALARM_LOG(Structure):
    _pack_ = 1  # 1바이트 경계로 정렬
    _fields_ = [("nAlarmCount", c_ubyte), ("nAlarmLog", c_ubyte * 30)]


def CALARM_LOG_to_ALARM_LOG(c):
    return ALARM_LOG(nAlarmCount=c.nAlarmCount, nAlarmLog=list(c.nAlarmLog))


class CTRIGGER_INFO(Structure):
    _pack_ = 2  # 2바이트 경계로 정렬
    _fields_ = [
        ("wPeriod", c_ushort),
        ("wReserved1", c_ushort),
        ("wOnTime", c_ushort),
        ("wReserved2", c_ushort),
        ("wCount", c_uint)
    ]


def CTRIGGER_INFO_to_TRIGGER_INFO(c):
    return TRIGGER_INFO(
        wPeriod=c.wPeriod,
        wReserved1=c.wReserved1,
        wOnTime=c.wOnTime,
        wReserved2=c.wReserved2,
        wCount=c.wCount
    )


def TRIGGER_INFO_to_CTRIGGER_INFO(v):
    return CTRIGGER_INFO(
        wPeriod=v.wPeriod,
        wReserved1=v.wReserved1,
        wOnTime=v.wOnTime,
        wReserved2=v.wReserved2,
        wCount=v.wCount
    )


# ------------------------------------------------------------------------------
# 					Connection Functions
# ------------------------------------------------------------------------------
# FAS_Connect 함수 정의
FAS_Connect_Original = EziMOTIONPlusE_dll["FAS_Connect"]
FAS_Connect_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_int
]
FAS_Connect_Original.restype = ctypes.c_int


# FAS_ConnectTCP 함수 정의
FAS_ConnectTCP_Original = EziMOTIONPlusE_dll["FAS_ConnectTCP"]
FAS_ConnectTCP_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_int
]
FAS_ConnectTCP_Original.restype = ctypes.c_int


# FAS_IsBdIDExist 함수 정의
FAS_IsBdIDExist_Original = EziMOTIONPlusE_dll["FAS_IsBdIDExist"]
FAS_IsBdIDExist_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_ubyte)
]
FAS_IsBdIDExist_Original.restype = ctypes.c_int


# FAS_IsIPAddressExist 함수 정의
FAS_IsIPAddressExist_Original = EziMOTIONPlusE_dll["FAS_IsIPAddressExist"]
FAS_IsIPAddressExist_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int)
]
FAS_IsIPAddressExist_Original.restype = ctypes.c_int


# FAS_Reconnect 함수 정의
FAS_Reconnect_Original = EziMOTIONPlusE_dll["FAS_Reconnect"]
FAS_Reconnect_Original.argtypes = [ctypes.c_int]
FAS_Reconnect_Original.restype = ctypes.c_int


# FAS_SetAutoReconnect 함수 정의
FAS_SetAutoReconnect_Original = EziMOTIONPlusE_dll["FAS_SetAutoReconnect"]
FAS_SetAutoReconnect_Original.argtypes = [ctypes.c_int]


# FAS_Close 함수 정의
FAS_Close_Original = EziMOTIONPlusE_dll["FAS_Close"]
FAS_Close_Original.argtypes = [ctypes.c_int]


# FAS_IsSlaveExist 함수 정의
FAS_IsSlaveExist_Original = EziMOTIONPlusE_dll["FAS_IsSlaveExist"]
FAS_IsSlaveExist_Original.argtypes = [ctypes.c_int]
FAS_IsSlaveExist_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					Ethernet Address Functions
# ------------------------------------------------------------------------------
# FAS_GetEthernetAddr 함수 정의
FAS_GetEthernetAddr_Original = EziMOTIONPlusE_dll["FAS_GetEthernetAddr"]
FAS_GetEthernetAddr_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetEthernetAddr_Original.restype = ctypes.c_int


# FAS_SetEthernetAddr 함수 정의
FAS_SetEthernetAddr_Original = EziMOTIONPlusE_dll["FAS_SetEthernetAddr"]
FAS_SetEthernetAddr_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetEthernetAddr_Original.restype = ctypes.c_int


# FAS_GetMACAddress 함수 정의
FAS_GetMACAddress_Original = EziMOTIONPlusE_dll["FAS_GetMACAddress"]
FAS_GetMACAddress_Original.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_ulonglong)]
FAS_GetMACAddress_Original.restype = ctypes.c_int


# FAS_EnableTwoTCPPort 함수 정의
FAS_EnableTwoTCPPort_Original = EziMOTIONPlusE_dll["FAS_EnableTwoTCPPort"]
FAS_EnableTwoTCPPort_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_EnableTwoTCPPort_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					Log Functions
# ------------------------------------------------------------------------------

# FAS_EnableLog 함수 정의 
FAS_EnableLog_Original = EziMOTIONPlusE_dll["FAS_EnableLog"]
FAS_EnableLog_Original.argtypes = [ctypes.c_int]


# FAS_SetLogLevel 함수 정의
FAS_SetLogLevel_Original = EziMOTIONPlusE_dll["FAS_SetLogLevel"]
FAS_SetLogLevel_Original.argtypes = [ctypes.c_int]


# FAS_SetLogPath 함수 정의 
FAS_SetLogPath_Original = EziMOTIONPlusE_dll["FAS_SetLogPath"]
FAS_SetLogPath_Original.argtypes = [c_wchar_p_type]
FAS_SetLogPath_Original.restype = ctypes.c_int


# FAS_PrintCustomLog 함수 정의 
FAS_PrintCustomLog_Original = EziMOTIONPlusE_dll["FAS_PrintCustomLog"]
FAS_PrintCustomLog_Original.argtypes = [ctypes.c_int, ctypes.c_int, c_wchar_p_type]


# ------------------------------------------------------------------------------
# 					Info Functions
# ------------------------------------------------------------------------------

# FAS_GetSlaveInfo 함수 정의
FAS_GetSlaveInfo_Original = EziMOTIONPlusE_dll["FAS_GetSlaveInfo"]
FAS_GetSlaveInfo_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_char_p,
    ctypes.c_int
]
FAS_GetSlaveInfo_Original.restype = ctypes.c_int


# FAS_GetMotorInfo 함수 정의
FAS_GetMotorInfo_Original = EziMOTIONPlusE_dll["FAS_GetMotorInfo"]
FAS_GetMotorInfo_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_char_p,
    ctypes.c_int
]
FAS_GetMotorInfo_Original.restype = ctypes.c_int


# FAS_GetSlaveInfoEx 함수 정의
FAS_GetSlaveInfoEx_Original = EziMOTIONPlusE_dll["FAS_GetSlaveInfoEx"]
FAS_GetSlaveInfoEx_Original.argtypes = [ctypes.c_int, POINTER(CDrive_Info)]
FAS_GetSlaveInfoEx_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					Parameter Functions
# ------------------------------------------------------------------------------
# FAS_SaveAllParameters 함수정의
FAS_SaveAllParameters_Original = EziMOTIONPlusE_dll["FAS_SaveAllParameters"]
FAS_SaveAllParameters_Original.argtypes = [ctypes.c_int]
FAS_SaveAllParameters_Original.restype = ctypes.c_int


# FAS_SetParameter 함수정의
FAS_SetParameter_Original = EziMOTIONPlusE_dll["FAS_SetParameter"]
FAS_SetParameter_Original.argtypes = [ctypes.c_int, ctypes.c_ubyte, ctypes.c_int]
FAS_SetParameter_Original.restype = ctypes.c_int


# FAS_GetParameter 함수정의
FAS_GetParameter_Original = EziMOTIONPlusE_dll["FAS_GetParameter"]
FAS_GetParameter_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetParameter_Original.restype = ctypes.c_int


# FAS_GetROMParameter 함수정의
FAS_GetROMParameter_Original = EziMOTIONPlusE_dll["FAS_GetROMParameter"]
FAS_GetROMParameter_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetROMParameter_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					IO Functions
# ------------------------------------------------------------------------------
# FAS_SetIOInput 함수정의
FAS_SetIOInput_Original = EziMOTIONPlusE_dll["FAS_SetIOInput"]
FAS_SetIOInput_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetIOInput_Original.restype = ctypes.c_int


# FAS_GetIOInput 함수정의
FAS_GetIOInput_Original = EziMOTIONPlusE_dll["FAS_GetIOInput"]
FAS_GetIOInput_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetIOInput_Original.restype = ctypes.c_int


# FAS_SetIOOutput 함수정의
FAS_SetIOOutput_Original = EziMOTIONPlusE_dll["FAS_SetIOOutput"]
FAS_SetIOOutput_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetIOOutput_Original.restype = ctypes.c_int


# FAS_GetIOOutput 함수정의
FAS_GetIOOutput_Original = EziMOTIONPlusE_dll["FAS_GetIOOutput"]
FAS_GetIOOutput_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetIOOutput_Original.restype = ctypes.c_int


# FAS_GetIOAssignMap 함수정의
FAS_GetIOAssignMap_Original = EziMOTIONPlusE_dll["FAS_GetIOAssignMap"]
FAS_GetIOAssignMap_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_ubyte)
]
FAS_GetIOAssignMap_Original.restype = ctypes.c_int


# FAS_SetIOAssignMap 함수정의
FAS_SetIOAssignMap_Original = EziMOTIONPlusE_dll["FAS_SetIOAssignMap"]
FAS_SetIOAssignMap_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_uint,
    ctypes.c_ubyte
]
FAS_SetIOAssignMap_Original.restype = ctypes.c_int


# FAS_IOAssignMapReadROM 함수정의
FAS_IOAssignMapReadROM_Original = EziMOTIONPlusE_dll["FAS_IOAssignMapReadROM"]
FAS_IOAssignMapReadROM_Original.argtypes = [ctypes.c_int]
FAS_IOAssignMapReadROM_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					Servo Driver Control Functions
# ------------------------------------------------------------------------------

# FAS_ServoEnable 함수정의
FAS_ServoEnable_Original = EziMOTIONPlusE_dll["FAS_ServoEnable"]
FAS_ServoEnable_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_ServoEnable_Original.restype = ctypes.c_int


# FAS_ServoAlarmReset 함수정의
FAS_ServoAlarmReset_Original = EziMOTIONPlusE_dll["FAS_ServoAlarmReset"]
FAS_ServoAlarmReset_Original.argtypes = [ctypes.c_int]
FAS_ServoAlarmReset_Original.restype = ctypes.c_int


# FAS_StepAlarmReset 함수정의
FAS_StepAlarmReset_Original = EziMOTIONPlusE_dll["FAS_StepAlarmReset"]
FAS_StepAlarmReset_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_StepAlarmReset_Original.restype = ctypes.c_int


# FAS_BrakeSet 함수정의
FAS_BrakeSet_Original = EziMOTIONPlusE_dll["FAS_BrakeSet"]
FAS_BrakeSet_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_BrakeSet_Original.restype = ctypes.c_int


# ------------------------------------------------------------------------------
# 					Read Status and Position
# ------------------------------------------------------------------------------

# FAS_GetAxisStatus 함수정의
FAS_GetAxisStatus_Original = EziMOTIONPlusE_dll["FAS_GetAxisStatus"]
FAS_GetAxisStatus_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetAxisStatus_Original.restype = ctypes.c_int


# FAS_GetIOAxisStatus 함수정의
FAS_GetIOAxisStatus_Original = EziMOTIONPlusE_dll["FAS_GetIOAxisStatus"]
FAS_GetIOAxisStatus_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetIOAxisStatus_Original.restype = ctypes.c_int


# FAS_GetMotionStatus 함수정의
FAS_GetMotionStatus_Original = EziMOTIONPlusE_dll["FAS_GetMotionStatus"]
FAS_GetMotionStatus_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_ushort)
]
FAS_GetMotionStatus_Original.restype = ctypes.c_int


# FAS_GetAllStatus 함수정의
FAS_GetAllStatus_Original = EziMOTIONPlusE_dll["FAS_GetAllStatus"]
FAS_GetAllStatus_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_ushort)
]
FAS_GetAllStatus_Original.restype = ctypes.c_int


# FAS_GetAllStatusEx 함수정의
FAS_GetAllStatusEx_Original = EziMOTIONPlusE_dll["FAS_GetAllStatusEx"]
FAS_GetAllStatusEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetAllStatusEx_Original.restype = ctypes.c_int


# FAS_SetCommandPos 함수정의
FAS_SetCommandPos_Original = EziMOTIONPlusE_dll["FAS_SetCommandPos"]
FAS_SetCommandPos_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_SetCommandPos_Original.restype = ctypes.c_int


# FAS_SetActualPos 함수정의
FAS_SetActualPos_Original = EziMOTIONPlusE_dll["FAS_SetActualPos"]
FAS_SetActualPos_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_SetActualPos_Original.restype = ctypes.c_int


# FAS_ClearPosition 함수정의
FAS_ClearPosition_Original = EziMOTIONPlusE_dll["FAS_ClearPosition"]
FAS_ClearPosition_Original.argtypes = [ctypes.c_int]
FAS_ClearPosition_Original.restype = ctypes.c_int


# FAS_GetCommandPos 함수정의
FAS_GetCommandPos_Original = EziMOTIONPlusE_dll["FAS_GetCommandPos"]
FAS_GetCommandPos_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetCommandPos_Original.restype = ctypes.c_int


# FAS_GetActualPos 함수 정의
FAS_GetActualPos_Original = EziMOTIONPlusE_dll["FAS_GetActualPos"]
FAS_GetActualPos_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetActualPos_Original.restype = ctypes.c_int


# FAS_GetPosError 함수 정의
FAS_GetPosError_Original = EziMOTIONPlusE_dll["FAS_GetPosError"]
FAS_GetPosError_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetPosError_Original.restype = ctypes.c_int



# FAS_GetActualVel 함수 정의
FAS_GetActualVel_Original = EziMOTIONPlusE_dll["FAS_GetActualVel"]
FAS_GetActualVel_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]


# FAS_GetAlarmType 함수 정의
FAS_GetAlarmType_Original = EziMOTIONPlusE_dll["FAS_GetAlarmType"]
FAS_GetAlarmType_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte)
]
FAS_GetAlarmType_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Motion Functions.
# ------------------------------------------------------------------
# FAS_MoveStop 함수 정의
FAS_MoveStop_Original = EziMOTIONPlusE_dll["FAS_MoveStop"]
FAS_MoveStop_Original.argtypes = [ctypes.c_int]
FAS_MoveStop_Original.restype = ctypes.c_int


# FAS_EmergencyStop 함수 정의
FAS_EmergencyStop_Original = EziMOTIONPlusE_dll["FAS_EmergencyStop"]
FAS_EmergencyStop_Original.argtypes = [ctypes.c_int]
FAS_EmergencyStop_Original.restype = ctypes.c_int


# FAS_MovePause 함수 정의
FAS_MovePause_Original = EziMOTIONPlusE_dll["FAS_MovePause"]
FAS_MovePause_Original.argtypes = [ctypes.c_int, ctypes.c_int]
FAS_MovePause_Original.restype = ctypes.c_int


# FAS_MoveOriginSingleAxis 함수 정의
FAS_MoveOriginSingleAxis_Original = EziMOTIONPlusE_dll["FAS_MoveOriginSingleAxis"]
FAS_MoveOriginSingleAxis_Original.argtypes = [ctypes.c_int]
FAS_MoveOriginSingleAxis_Original.restype = ctypes.c_int


# FAS_MoveSingleAxisAbsPos 함수 정의
FAS_MoveSingleAxisAbsPos_Original = EziMOTIONPlusE_dll["FAS_MoveSingleAxisAbsPos"]
FAS_MoveSingleAxisAbsPos_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint
]
FAS_MoveSingleAxisAbsPos_Original.restype = ctypes.c_int


# FAS_MoveSingleAxisIncPos 함수 정의
FAS_MoveSingleAxisIncPos_Original = EziMOTIONPlusE_dll["FAS_MoveSingleAxisIncPos"]
FAS_MoveSingleAxisIncPos_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint
]
FAS_MoveSingleAxisIncPos_Original.restype = ctypes.c_int


# FAS_MoveToLimit 함수 정의
FAS_MoveToLimit_Original = EziMOTIONPlusE_dll["FAS_MoveToLimit"]
FAS_MoveToLimit_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_int
]
FAS_MoveToLimit_Original.restype = ctypes.c_int


# FAS_MoveVelocity 함수 정의
FAS_MoveVelocity_Original = EziMOTIONPlusE_dll["FAS_MoveVelocity"]
FAS_MoveVelocity_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_int
]
FAS_MoveVelocity_Original.restype = ctypes.c_int


# FAS_PositionAbsOverride 함수 정의
FAS_PositionAbsOverride_Original = EziMOTIONPlusE_dll["FAS_PositionAbsOverride"]
FAS_PositionAbsOverride_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int
]
FAS_PositionAbsOverride_Original.restype = ctypes.c_int


# FAS_PositionIncOverride 함수 정의
FAS_PositionIncOverride_Original = EziMOTIONPlusE_dll["FAS_PositionIncOverride"]
FAS_PositionIncOverride_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int
]
FAS_PositionIncOverride_Original.restype = ctypes.c_int


# FAS_VelocityOverride 함수 정의
FAS_VelocityOverride_Original = EziMOTIONPlusE_dll["FAS_VelocityOverride"]
FAS_VelocityOverride_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint
]
FAS_VelocityOverride_Original.restype = ctypes.c_int


# FAS_MoveLinearAbsPos 함수 정의
FAS_MoveLinearAbsPos_Original = EziMOTIONPlusE_dll["FAS_MoveLinearAbsPos"]
FAS_MoveLinearAbsPos_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_ushort
]
FAS_MoveLinearAbsPos_Original.restype = ctypes.c_int


# FAS_MoveLinearIncPos 함수 정의
FAS_MoveLinearIncPos_Original = EziMOTIONPlusE_dll["FAS_MoveLinearIncPos"]
FAS_MoveLinearIncPos_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_ushort
]
FAS_MoveLinearIncPos_Original.restype = ctypes.c_int


# FAS_MoveLinearAbsPos2 함수 정의
FAS_MoveLinearAbsPos2_Original = EziMOTIONPlusE_dll["FAS_MoveLinearAbsPos2"]
FAS_MoveLinearAbsPos2_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_ushort
]
FAS_MoveLinearAbsPos2_Original.restype = ctypes.c_int


# FAS_MoveLinearIncPos2 함수 정의
FAS_MoveLinearIncPos2_Original = EziMOTIONPlusE_dll["FAS_MoveLinearIncPos2"]
FAS_MoveLinearIncPos2_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_ushort
]
FAS_MoveLinearIncPos2_Original.restype = ctypes.c_int


# FAS_MoveCircleAbsPos1 함수 정의
FAS_MoveCircleAbsPos1_Original = EziMOTIONPlusE_dll["FAS_MoveCircleAbsPos1"]
FAS_MoveCircleAbsPos1_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleAbsPos1_Original.restype = ctypes.c_int


# FAS_MoveCircleIncPos1 함수 정의
FAS_MoveCircleIncPos1_Original = EziMOTIONPlusE_dll["FAS_MoveCircleIncPos1"]
FAS_MoveCircleIncPos1_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleIncPos1_Original.restype = ctypes.c_int


# FAS_MoveCircleAbsPos2 함수 정의
FAS_MoveCircleAbsPos2_Original = EziMOTIONPlusE_dll["FAS_MoveCircleAbsPos2"]
FAS_MoveCircleAbsPos2_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleAbsPos2_Original.restype = ctypes.c_int


# FAS_MoveCircleIncPos2 함수 정의
FAS_MoveCircleIncPos2_Original = EziMOTIONPlusE_dll["FAS_MoveCircleIncPos2"]
FAS_MoveCircleIncPos2_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleIncPos2_Original.restype = ctypes.c_int


# FAS_MoveCircleAbsPos3 함수 정의
FAS_MoveCircleAbsPos3_Original = EziMOTIONPlusE_dll["FAS_MoveCircleAbsPos3"]
FAS_MoveCircleAbsPos3_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleAbsPos3_Original.restype = ctypes.c_int


# FAS_MoveCircleIncPos3 함수 정의
FAS_MoveCircleIncPos3_Original = EziMOTIONPlusE_dll["FAS_MoveCircleIncPos3"]
FAS_MoveCircleIncPos3_Original.argtypes = [
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_MoveCircleIncPos3_Original.restype = ctypes.c_int


# FAS_TriggerOutput_RunA 함수 정의
FAS_TriggerOutput_RunA_Original = EziMOTIONPlusE_dll["FAS_TriggerOutput_RunA"]
FAS_TriggerOutput_RunA_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_TriggerOutput_RunA_Original.restype = ctypes.c_int


# FAS_TriggerOutput_Status 함수 정의
FAS_TriggerOutput_Status_Original = EziMOTIONPlusE_dll["FAS_TriggerOutput_Status"]
FAS_TriggerOutput_Status_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte)
]
FAS_TriggerOutput_Status_Original.restype = ctypes.c_int


# FAS_SetTriggerOutputEx 함수 정의
FAS_SetTriggerOutputEx_Original = EziMOTIONPlusE_dll["FAS_SetTriggerOutputEx"]
FAS_SetTriggerOutputEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.c_ushort,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int)
]
FAS_SetTriggerOutputEx_Original.restype = ctypes.c_int


# FAS_GetTriggerOutputEx 함수 정의
FAS_GetTriggerOutputEx_Original = EziMOTIONPlusE_dll["FAS_GetTriggerOutputEx"]
FAS_GetTriggerOutputEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_ushort),
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetTriggerOutputEx_Original.restype = ctypes.c_int


# FAS_MovePush 함수 정의
FAS_MovePush_Original = EziMOTIONPlusE_dll["FAS_MovePush"]
FAS_MovePush_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.c_ushort,
    ctypes.c_ushort,
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.c_ushort
]
FAS_MovePush_Original.restype = ctypes.c_int


# FAS_GetPushStatus 함수 정의
FAS_GetPushStatus_Original = EziMOTIONPlusE_dll["FAS_GetPushStatus"]
FAS_GetPushStatus_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte)
]
FAS_GetPushStatus_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Ex-Motion Functions.
# ------------------------------------------------------------------
# FAS_MoveSingleAxisAbsPosEx 함수 정의
FAS_MoveSingleAxisAbsPosEx_Original = EziMOTIONPlusE_dll["FAS_MoveSingleAxisAbsPosEx"]
FAS_MoveSingleAxisAbsPosEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.POINTER(CMOTION_OPTION_EX),
]
FAS_MoveSingleAxisAbsPosEx_Original.restype = ctypes.c_int


# FAS_MoveSingleAxisIncPosEx 함수 정의
FAS_MoveSingleAxisIncPosEx_Original = EziMOTIONPlusE_dll["FAS_MoveSingleAxisIncPosEx"]
FAS_MoveSingleAxisIncPosEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.POINTER(CMOTION_OPTION_EX),
]
FAS_MoveSingleAxisIncPosEx_Original.restype = ctypes.c_int


# FAS_MoveVelocityEx 함수 정의
FAS_MoveVelocityEx_Original = EziMOTIONPlusE_dll["FAS_MoveVelocityEx"]
FAS_MoveVelocityEx_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_int,
    ctypes.POINTER(CVELOCITY_OPTION_EX),
]
FAS_MoveVelocityEx_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Position Table Functions.
# ------------------------------------------------------------------

# FAS_PosTableReadItem 함수 정의
FAS_PosTableReadItem_Original = EziMOTIONPlusE_dll["FAS_PosTableReadItem"]
FAS_PosTableReadItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.POINTER(CITEM_NODE)
]
FAS_PosTableReadItem_Original.restype = ctypes.c_int


# FAS_PosTableWriteItem 함수 정의
FAS_PosTableWriteItem_Original = EziMOTIONPlusE_dll["FAS_PosTableWriteItem"]
FAS_PosTableWriteItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.POINTER(CITEM_NODE)
]
FAS_PosTableWriteItem_Original.restype = ctypes.c_int


# FAS_PosTableWriteROM 함수 정의
FAS_PosTableWriteROM_Original = EziMOTIONPlusE_dll["FAS_PosTableWriteROM"]
FAS_PosTableWriteROM_Original.argtypes = [ctypes.c_int]
FAS_PosTableWriteROM_Original.restype = ctypes.c_int


# FAS_PosTableReadROM 함수 정의
FAS_PosTableReadROM_Original = EziMOTIONPlusE_dll["FAS_PosTableReadROM"]
FAS_PosTableReadROM_Original.argtypes = [ctypes.c_int]
FAS_PosTableReadROM_Original.restype = ctypes.c_int


# FAS_PosTableRunItem 함수 정의
FAS_PosTableRunItem_Original = EziMOTIONPlusE_dll["FAS_PosTableRunItem"]
FAS_PosTableRunItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort
]
FAS_PosTableRunItem_Original.restype = ctypes.c_int


# FAS_PosTableReadOneItem 함수 정의
FAS_PosTableReadOneItem_Original = EziMOTIONPlusE_dll["FAS_PosTableReadOneItem"]
FAS_PosTableReadOneItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.c_ushort,
    ctypes.POINTER(ctypes.c_int)
]
FAS_PosTableReadOneItem_Original.restype = ctypes.c_int



# FAS_PosTableWriteOneItem 함수 정의
FAS_PosTableWriteOneItem_Original = EziMOTIONPlusE_dll["FAS_PosTableWriteOneItem"]
FAS_PosTableWriteOneItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.c_ushort,
    ctypes.c_int
]
FAS_PosTableWriteOneItem_Original.restype = ctypes.c_int


# FAS_PosTableSingleRunItem 함수 정의
FAS_PosTableSingleRunItem_Original = EziMOTIONPlusE_dll["FAS_PosTableSingleRunItem"]
FAS_PosTableSingleRunItem_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_ushort
]
FAS_PosTableSingleRunItem_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Gap Control Functions.
# ------------------------------------------------------------------
# FAS_GapControlEnable 함수 정의
FAS_GapControlEnable_Original = EziMOTIONPlusE_dll["FAS_GapControlEnable"]
FAS_GapControlEnable_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ushort,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int
]
FAS_GapControlEnable_Original.restype = ctypes.c_int


# FAS_GapControlDisable 함수 정의
FAS_GapControlDisable_Original = EziMOTIONPlusE_dll["FAS_GapControlDisable"]
FAS_GapControlDisable_Original.argtypes = [ctypes.c_int]
FAS_GapControlDisable_Original.restype = ctypes.c_int


# FAS_IsGapControlEnable 함수 정의
FAS_IsGapControlEnable_Original = EziMOTIONPlusE_dll["FAS_IsGapControlEnable"]
FAS_IsGapControlEnable_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_ushort)
]
FAS_IsGapControlEnable_Original.restype = ctypes.c_int


# FAS_GapControlGetADCValue 함수 정의
FAS_GapControlGetADCValue_Original = EziMOTIONPlusE_dll["FAS_GapControlGetADCValue"]
FAS_GapControlGetADCValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GapControlGetADCValue_Original.restype = ctypes.c_int


# FAS_GapOneResultMonitor 함수 정의
FAS_GapOneResultMonitor_Original = EziMOTIONPlusE_dll["FAS_GapOneResultMonitor"]
FAS_GapOneResultMonitor_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int)
]
FAS_GapOneResultMonitor_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Alarm Type History Functions.
# ------------------------------------------------------------------
# FAS_GetAlarmLogs 함수 정의
FAS_GetAlarmLogs_Original = EziMOTIONPlusE_dll["FAS_GetAlarmLogs"]
FAS_GetAlarmLogs_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(CALARM_LOG)
]
FAS_GetAlarmLogs_Original.restype = ctypes.c_int


# FAS_ResetAlarmLogs 함수 정의
FAS_ResetAlarmLogs_Original = EziMOTIONPlusE_dll["FAS_ResetAlarmLogs"]
FAS_ResetAlarmLogs_Original.argtypes = [ctypes.c_int]
FAS_ResetAlarmLogs_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					I/O Module Functions.
# ------------------------------------------------------------------
# FAS_GetInput 함수 정의
FAS_GetInput_Original = EziMOTIONPlusE_dll["FAS_GetInput"]
FAS_GetInput_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetInput_Original.restype = ctypes.c_int


# FAS_ClearLatch 함수 정의
FAS_ClearLatch_Original = EziMOTIONPlusE_dll["FAS_ClearLatch"]
FAS_ClearLatch_Original.argtypes = [ctypes.c_int, ctypes.c_uint]
FAS_ClearLatch_Original.restype = ctypes.c_int


# FAS_GetLatchCount 함수 정의
FAS_GetLatchCount_Original = EziMOTIONPlusE_dll["FAS_GetLatchCount"]
FAS_GetLatchCount_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetLatchCount_Original.restype = ctypes.c_int


# FAS_GetLatchCountAll 함수 정의
FAS_GetLatchCountAll_Original = EziMOTIONPlusE_dll["FAS_GetLatchCountAll"]
FAS_GetLatchCountAll_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint * 16)
]
FAS_GetLatchCountAll_Original.restype = ctypes.c_int


# FAS_GetLatchCountAll32 함수 정의
FAS_GetLatchCountAll32_Original = EziMOTIONPlusE_dll["FAS_GetLatchCountAll32"]
FAS_GetLatchCountAll32_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint * 32)
]
FAS_GetLatchCountAll32_Original.restype = ctypes.c_int


# FAS_ClearLatchCount 함수 정의
FAS_ClearLatchCount_Original = EziMOTIONPlusE_dll["FAS_ClearLatchCount"]
FAS_ClearLatchCount_Original.argtypes = [ctypes.c_int, ctypes.c_uint]
FAS_ClearLatchCount_Original.restype = ctypes.c_int


# FAS_GetOutput 함수 정의
FAS_GetOutput_Original = EziMOTIONPlusE_dll["FAS_GetOutput"]
FAS_GetOutput_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint),
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetOutput_Original.restype = ctypes.c_int


# FAS_SetOutput 함수 정의
FAS_SetOutput_Original = EziMOTIONPlusE_dll["FAS_SetOutput"]
FAS_SetOutput_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetOutput_Original.restype = ctypes.c_int


# FAS_SetTrigger 함수 정의
FAS_SetTrigger_Original = EziMOTIONPlusE_dll["FAS_SetTrigger"]
FAS_SetTrigger_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(CTRIGGER_INFO)
]
FAS_SetTrigger_Original.restype = ctypes.c_int


# FAS_SetRunStop 함수 정의
FAS_SetRunStop_Original = EziMOTIONPlusE_dll["FAS_SetRunStop"]
FAS_SetRunStop_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetRunStop_Original.restype = ctypes.c_int


# FAS_GetTriggerCount 함수 정의
FAS_GetTriggerCount_Original = EziMOTIONPlusE_dll["FAS_GetTriggerCount"]
FAS_GetTriggerCount_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetTriggerCount_Original.restype = ctypes.c_int


# FAS_GetIOLevel 함수 정의
FAS_GetIOLevel_Original = EziMOTIONPlusE_dll["FAS_GetIOLevel"]
FAS_GetIOLevel_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetIOLevel_Original.restype = ctypes.c_int


# FAS_SetIOLevel 함수 정의
FAS_SetIOLevel_Original = EziMOTIONPlusE_dll["FAS_SetIOLevel"]
FAS_SetIOLevel_Original.argtypes = [ctypes.c_int, ctypes.c_uint]
FAS_SetIOLevel_Original.restype = ctypes.c_int


# FAS_LoadIOLevel 함수 정의
FAS_LoadIOLevel_Original = EziMOTIONPlusE_dll["FAS_LoadIOLevel"]
FAS_LoadIOLevel_Original.argtypes = [ctypes.c_int]
FAS_LoadIOLevel_Original.restype = ctypes.c_int


# FAS_SaveIOLevel 함수 정의
FAS_SaveIOLevel_Original = EziMOTIONPlusE_dll["FAS_SaveIOLevel"]
FAS_SaveIOLevel_Original.argtypes = [ctypes.c_int]
FAS_SaveIOLevel_Original.restype = ctypes.c_int


# FAS_GetInputFilter 함수 정의
FAS_GetInputFilter_Original = EziMOTIONPlusE_dll["FAS_GetInputFilter"]
FAS_GetInputFilter_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_ushort)
]
FAS_GetInputFilter_Original.restype = ctypes.c_int


# FAS_SetInputFilter 함수 정의
FAS_SetInputFilter_Original = EziMOTIONPlusE_dll["FAS_SetInputFilter"]
FAS_SetInputFilter_Original.argtypes = [ctypes.c_int, ctypes.c_ushort]
FAS_SetInputFilter_Original.restype = ctypes.c_int


# FAS_GetIODirection 함수 정의
FAS_GetIODirection_Original = EziMOTIONPlusE_dll["FAS_GetIODirection"]
FAS_GetIODirection_Original.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint)
]
FAS_GetIODirection_Original.restype = ctypes.c_int


# FAS_SetIODirection 함수 정의
FAS_SetIODirection_Original = EziMOTIONPlusE_dll["FAS_SetIODirection"]
FAS_SetIODirection_Original.argtypes = [ctypes.c_int, ctypes.c_uint]
FAS_SetIODirection_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 					Ezi-IO AD Functions
# ------------------------------------------------------------------
# FAS_SetADConfig 함수 정의 
FAS_SetADConfig_Original = EziMOTIONPlusE_dll["FAS_SetADConfig"]
FAS_SetADConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_SetADConfig_Original.restype = ctypes.c_int


# FAS_GetADConfig 함수 정의 
FAS_GetADConfig_Original = EziMOTIONPlusE_dll["FAS_GetADConfig"]
FAS_GetADConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetADConfig_Original.restype = ctypes.c_int


# FAS_LoadADConfig 함수 정의
FAS_LoadADConfig_Original = EziMOTIONPlusE_dll["FAS_LoadADConfig"]
FAS_LoadADConfig_Original.argtypes = [ctypes.c_int]
FAS_LoadADConfig_Original.restype = ctypes.c_int


# FAS_SaveADConfig 함수 정의
FAS_SaveADConfig_Original = EziMOTIONPlusE_dll["FAS_SaveADConfig"]
FAS_SaveADConfig_Original.argtypes = [ctypes.c_int]
FAS_SaveADConfig_Original.restype = ctypes.c_int


# FAS_ReadADValue 함수 정의
FAS_ReadADValue_Original = EziMOTIONPlusE_dll["FAS_ReadADValue"]
FAS_ReadADValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_short)
]
FAS_ReadADValue_Original.restype = ctypes.c_int


# FAS_ReadADAllValue 함수 정의
FAS_ReadADAllValue_Original = EziMOTIONPlusE_dll["FAS_ReadADAllValue"]
FAS_ReadADAllValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_short * 8)
]
FAS_ReadADAllValue_Original.restype = ctypes.c_int


# FAS_GetAllADResult 함수 정의
FAS_GetAllADResult_Original = EziMOTIONPlusE_dll["FAS_GetAllADResult"]
FAS_GetAllADResult_Original.argtypes = [ctypes.c_int, ctypes.POINTER(AD_RESULT)]
FAS_GetAllADResult_Original.restype = ctypes.c_int


# FAS_GetADResult 함수 정의
FAS_GetADResult_Original = EziMOTIONPlusE_dll["FAS_GetADResult"]
FAS_GetADResult_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_float)
]
FAS_GetADResult_Original.restype = ctypes.c_int


# FAS_SetADRange 함수 정의 
FAS_SetADRange_Original = EziMOTIONPlusE_dll["FAS_SetADRange"]
FAS_SetADRange_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int
]
FAS_SetADRange_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 			Ezi-IO DA Functions
# ------------------------------------------------------------------
# FAS_SetDACConfig 함수 정의 
FAS_SetDACConfig_Original = EziMOTIONPlusE_dll["FAS_SetDACConfig"]
FAS_SetDACConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_SetDACConfig_Original.restype = ctypes.c_int


# FAS_GetDACConfig 함수 정의 
FAS_GetDACConfig_Original = EziMOTIONPlusE_dll["FAS_GetDACConfig"]
FAS_GetDACConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
]
FAS_GetDACConfig_Original.restype = ctypes.c_int


# FAS_LoadDACConfig 함수 정의
FAS_LoadDACConfig_Original = EziMOTIONPlusE_dll["FAS_LoadDACConfig"]
FAS_LoadDACConfig_Original.argtypes = [ctypes.c_int]
FAS_LoadDACConfig_Original.restype = ctypes.c_int


# FAS_SaveDACConfig 함수 정의
FAS_SaveDACConfig_Original = EziMOTIONPlusE_dll["FAS_SaveDACConfig"]
FAS_SaveDACConfig_Original.argtypes = [ctypes.c_int]
FAS_SaveDACConfig_Original.restype = ctypes.c_int


# FAS_SetDACValue 함수 정의
FAS_SetDACValue_Original = EziMOTIONPlusE_dll["FAS_SetDACValue"]
FAS_SetDACValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int
]
FAS_SetDACValue_Original.restype = ctypes.c_int


# FAS_GetDACValue 함수 정의
FAS_GetDACValue_Original = EziMOTIONPlusE_dll["FAS_GetDACValue"]
FAS_GetDACValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetDACValue_Original.restype = ctypes.c_int


# ------------------------------------------------------------------
# 			Ezi-IO Counter Functions
# ------------------------------------------------------------------
# FAS_CounterCommand 함수 정의
FAS_CounterCommand_Original = EziMOTIONPlusE_dll["FAS_CounterCommand"]
FAS_CounterCommand_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int
]
FAS_CounterCommand_Original.restype = ctypes.c_int


# FAS_GetCounterValue 함수 정의
FAS_GetCounterValue_Original = EziMOTIONPlusE_dll["FAS_GetCounterValue"]
FAS_GetCounterValue_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetCounterValue_Original.restype = ctypes.c_int


# FAS_GetCounterStatus 함수 정의
FAS_GetCounterStatus_Original = EziMOTIONPlusE_dll["FAS_GetCounterStatus"]
FAS_GetCounterStatus_Original.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
FAS_GetCounterStatus_Original.restype = ctypes.c_int


# FAS_SetCounterTrigger 함수 정의
FAS_SetCounterTrigger_Original = EziMOTIONPlusE_dll["FAS_SetCounterTrigger"]
FAS_SetCounterTrigger_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_uint,
    ctypes.c_uint,
    ctypes.c_uint
]
FAS_SetCounterTrigger_Original.restype = ctypes.c_int


# FAS_SetCounterConfig 함수 정의 
FAS_SetCounterConfig_Original = EziMOTIONPlusE_dll["FAS_SetCounterConfig"]
FAS_SetCounterConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_SetCounterConfig_Original.restype = ctypes.c_int


# FAS_GetCounterConfig 함수 정의 
FAS_GetCounterConfig_Original = EziMOTIONPlusE_dll["FAS_GetCounterConfig"]
FAS_GetCounterConfig_Original.argtypes = [
    ctypes.c_int,
    ctypes.c_ubyte,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
FAS_GetCounterConfig_Original.restype = ctypes.c_int


# FAS_LoadCounterConfig 함수 정의
FAS_LoadCounterConfig_Original = EziMOTIONPlusE_dll["FAS_LoadCounterConfig"]
FAS_LoadCounterConfig_Original.argtypes = [ctypes.c_int]
FAS_LoadCounterConfig_Original.restype = ctypes.c_int


# FAS_SaveCounterConfig 함수 정의
FAS_SaveCounterConfig_Original = EziMOTIONPlusE_dll["FAS_SaveCounterConfig"]
FAS_SaveCounterConfig_Original.argtypes = [ctypes.c_int]
FAS_SaveCounterConfig_Original.restype = ctypes.c_int
