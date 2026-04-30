#####################################################
# Version: 1.0.0                                    #
# Last modified date: 25.10.31                      #
# Changes log: First released version               #
#####################################################


# -*- coding: utf-8 -*-
from ctypes import *
import ctypes
from FAS_EziMOTIONPlusE_Internal import *
from MOTION_DEFINE import *



# ------------------------------------------------------------------------------
# 					Connection Functions
# ------------------------------------------------------------------------------

# def FAS_Connect(sb1: int, sb2: int, sb3: int, sb4: int, iBdID: int) -> int:
def FAS_Connect(sb1, sb2, sb3, sb4, iBdID):
    return FAS_Connect_Original(
        ctypes.c_ubyte(sb1),
        ctypes.c_ubyte(sb2),
        ctypes.c_ubyte(sb3),
        ctypes.c_ubyte(sb4),
        ctypes.c_int(iBdID)
    )
# def FAS_ConnectTCP(sb1: int, sb2: int, sb3: int, sb4: int, iBdID: int) -> int:
def FAS_ConnectTCP(sb1, sb2, sb3, sb4, iBdID):
    return FAS_ConnectTCP_Original(
        ctypes.c_ubyte(sb1),
        ctypes.c_ubyte(sb2),
        ctypes.c_ubyte(sb3),
        ctypes.c_ubyte(sb4),
        ctypes.c_int(iBdID)
    )

# def FAS_IsBdIDExist(iBdID: int) -> tuple[bool, int, int, int, int]:
def FAS_IsBdIDExist(iBdID):
    sb1 = ctypes.c_ubyte()
    sb2 = ctypes.c_ubyte()
    sb3 = ctypes.c_ubyte()
    sb4 = ctypes.c_ubyte()
    result = FAS_IsBdIDExist_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(sb1),
        ctypes.byref(sb2),
        ctypes.byref(sb3),
        ctypes.byref(sb4)
    )
    if result == 0:
        return result, None, None, None, None
    else:
        return result, sb1.value, sb2.value, sb3.value, sb4.value

# def FAS_IsIPAddressExist(sb1: int, sb2: int, sb3: int, sb4: int) -> tuple[bool, int]:
def FAS_IsIPAddressExist(sb1, sb2, sb3, sb4):
    iBdID = ctypes.c_int()
    result = FAS_IsIPAddressExist_Original(
        ctypes.c_ubyte(sb1),
        ctypes.c_ubyte(sb2),
        ctypes.c_ubyte(sb3),
        ctypes.c_ubyte(sb4),
        ctypes.byref(iBdID)
    )
    if result == 0:
        return result, None
    else:
        return result, iBdID.value

# def FAS_Reconnect(iBdID: int) -> bool:
def FAS_Reconnect(iBdID):
    return FAS_Reconnect_Original(ctypes.c_int(iBdID))

# def FAS_SetAutoReconnect(bSET: bool):
def FAS_SetAutoReconnect(bSET):
    FAS_SetAutoReconnect_Original(ctypes.c_int(bSET))

# def FAS_Close(iBdID: int):
def FAS_Close(iBdID):
    FAS_Close_Original(ctypes.c_int(iBdID))

# def FAS_IsSlaveExist(iBdID: int) -> bool:
def FAS_IsSlaveExist(iBdID):
    return FAS_IsSlaveExist_Original(ctypes.c_int(iBdID))


# ------------------------------------------------------------------------------
# 					Ethernet Address Functions
# ------------------------------------------------------------------------------
# def FAS_GetEthernetAddr(iBdID: int) -> tuple[int, int, int, int]:
def FAS_GetEthernetAddr(iBdID):
    gateway = ctypes.c_uint()
    subnet = ctypes.c_uint()
    ip = ctypes.c_uint()
    result = FAS_GetEthernetAddr_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(gateway),
        ctypes.byref(subnet),
        ctypes.byref(ip)
    )
    if result == 0:
        return result, gateway.value, subnet.value, ip.value
    else:
        return result, None, None, None


# def FAS_SetEthernetAddr(iBdID: int, gateway: int, subnet: int, ip: int) -> int:
def FAS_SetEthernetAddr(iBdID, gateway, subnet, ip):
    return FAS_SetEthernetAddr_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(gateway),
        ctypes.c_uint(subnet),
        ctypes.c_uint(ip)
    )


# def FAS_GetMACAddress(iBdID: int) -> tuple[int, int]:
def FAS_GetMACAddress(iBdID):
    MACAddress = ctypes.c_ulonglong()
    result = FAS_GetMACAddress_Original(ctypes.c_int(iBdID), ctypes.byref(MACAddress))
    if result == 0:
        return result, MACAddress.value
    else:
        return result, None


# def FAS_EnableTwoTCPPort(iBdID: int, bEnable: bool) -> int:
def FAS_EnableTwoTCPPort(iBdID, bEnable):
    return FAS_EnableTwoTCPPort_Original(ctypes.c_int(iBdID), ctypes.c_int(bEnable))


# ------------------------------------------------------------------------------
# 					Log Functions
# ------------------------------------------------------------------------------
# def FAS_EnableLog(bEnable: bool):
def FAS_EnableLog(bEnable):
    FAS_EnableLog_Original(ctypes.c_int(bEnable))


# def FAS_SetLogLevel(level: int):
def FAS_SetLogLevel(level):
    FAS_SetLogLevel_Original(ctypes.c_int(level))


# def FAS_SetLogPath(lpPath: str) -> bool:
def FAS_SetLogPath(lpPath):
    return FAS_SetLogPath_Original(to_unicode(lpPath))


# def FAS_PrintCustomLog(iBdID: int, level: int, lpszMsg: str):
def FAS_PrintCustomLog(iBdID, level, lpszMsg):
    FAS_PrintCustomLog_Original(
        ctypes.c_int(iBdID), ctypes.c_int(level), to_unicode(lpszMsg)
    )


# ------------------------------------------------------------------------------
# 					Info Functions
# ------------------------------------------------------------------------------
# def FAS_GetSlaveInfo(iBdID: int) -> tuple[int, int, str]:
def FAS_GetSlaveInfo(iBdID):
    pType = ctypes.c_ubyte()
    lpBuff = ctypes.create_string_buffer(256)

    result = FAS_GetSlaveInfo_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(pType),
        lpBuff,
        256
    )

    if result == 0:
        return result, pType.value, lpBuff.value.decode("utf-8")
    else:
        return result, None, None


# def FAS_GetMotorInfo(iBdID: int) -> tuple[int, int, str]:
def FAS_GetMotorInfo(iBdID):
    pType = ctypes.c_ubyte()
    lpBuff = ctypes.create_string_buffer(256)

    result = FAS_GetMotorInfo_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(pType),
        lpBuff,
        256
    )

    if result == 0:
        return result, pType.value, lpBuff.value.decode("utf-8")
    else:
        return result, None, None


# def FAS_GetSlaveInfoEx(iBdID: int) -> tuple[int, str]:
def FAS_GetSlaveInfoEx(iBdID):
    drive_info = CDrive_Info()
    result = FAS_GetSlaveInfoEx_Original(ctypes.c_int(iBdID), byref(drive_info))
    if result == 0:
        lpDriveInfo = CDrive_Info_to_Drive_Info(drive_info)
        return result, lpDriveInfo
    else:
        return result, None

# ------------------------------------------------------------------------------
# 					Parameter Functions
# ------------------------------------------------------------------------------
# def FAS_SaveAllParameters(iBdID: int) -> int:
def FAS_SaveAllParameters(iBdID):
    return FAS_SaveAllParameters_Original(ctypes.c_int(iBdID))


# def FAS_SetParameter(iBdID: int, iParamNo: int, lParamValue: int) -> int:
def FAS_SetParameter(iBdID, iParamNo, lParamValue):
    return FAS_SetParameter_Original(
        ctypes.c_int(iBdID), ctypes.c_ubyte(iParamNo), ctypes.c_int(lParamValue)
    )


# def FAS_GetParameter(iBdID: int, iParamNo: int) -> tuple[int, int]:
def FAS_GetParameter(iBdID, iParamNo):
    lParamValue = ctypes.c_int()
    result = FAS_GetParameter_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(iParamNo),
        ctypes.byref(lParamValue)
    )
    if result == 0:
        return result, lParamValue.value
    else:
        return result, None


# def FAS_GetROMParameter(iBdID: int, iParamNo: int) -> tuple[int, int]:
def FAS_GetROMParameter(iBdID, iParamNo):
    lRomParam = ctypes.c_int()
    result = FAS_GetROMParameter_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(iParamNo),
        ctypes.byref(lRomParam)
    )
    if result == 0:
        return result, lRomParam.value
    else:
        return result, None


# ------------------------------------------------------------------------------
# 					IO Functions
# ------------------------------------------------------------------------------
# def FAS_SetIOInput(iBdID: int, dwIOSETMask: int, dwIOCLRMask: int) -> int:
def FAS_SetIOInput(iBdID, dwIOSETMask, dwIOCLRMask):
    return FAS_SetIOInput_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(dwIOSETMask),
        ctypes.c_uint(dwIOCLRMask)
    )


# def FAS_GetIOInput(iBdID: int) -> tuple[int, int]:
def FAS_GetIOInput(iBdID):
    dwIOInput = ctypes.c_uint()
    result = FAS_GetIOInput_Original(ctypes.c_int(iBdID), ctypes.byref(dwIOInput))
    if result == 0:
        return result, dwIOInput.value
    else:
        return result, None


# def FAS_SetIOOutput(iBdID: int, dwIOSETMask: int, dwIOCLRMask: int) -> int:
def FAS_SetIOOutput(iBdID, dwIOSETMask, dwIOCLRMask):
    return FAS_SetIOOutput_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(dwIOSETMask),
        ctypes.c_uint(dwIOCLRMask)
    )


# def FAS_GetIOOutput(iBdID: int) -> tuple[int, int]:
def FAS_GetIOOutput(iBdID):
    dwIOOutput = ctypes.c_uint()
    result = FAS_GetIOOutput_Original(ctypes.c_int(iBdID), ctypes.byref(dwIOOutput))
    if result == 0:
        return result, dwIOOutput.value
    else:
        return result, None


# def FAS_GetIOAssignMap(iBdID: int, iIOPinNo: int) -> tuple[int, int, int]:
def FAS_GetIOAssignMap(iBdID, iIOPinNo):
    dwIOLogicMask = ctypes.c_uint()
    bLevel = ctypes.c_ubyte()
    result = FAS_GetIOAssignMap_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(iIOPinNo),
        ctypes.byref(dwIOLogicMask),
        ctypes.byref(bLevel)
    )
    if result == 0:
        return result, dwIOLogicMask.value, bLevel.value
    else:
        return result, None, None
# def FAS_SetIOAssignMap( iBdID: int, iIOPinNo: int, dwIOLogicMask: int, bLevel: int) -> int:
def FAS_SetIOAssignMap(
    iBdID, iIOPinNo, dwIOLogicMask, bLevel
):
    return FAS_SetIOAssignMap_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(iIOPinNo),
        ctypes.c_uint(dwIOLogicMask),
        ctypes.c_ubyte(bLevel)
    )


# def FAS_IOAssignMapReadROM(iBdID: int) -> int:
def FAS_IOAssignMapReadROM(iBdID):
    return FAS_IOAssignMapReadROM_Original(ctypes.c_int(iBdID))


# ------------------------------------------------------------------------------
# 					Servo Driver Control Functions
# ------------------------------------------------------------------------------


# def FAS_ServoEnable(iBdID: int, bOnOff: bool) -> int:
def FAS_ServoEnable(iBdID, bOnOff):
    return FAS_ServoEnable_Original(ctypes.c_int(iBdID), ctypes.c_int(bOnOff))


# def FAS_ServoAlarmReset(iBdID: int) -> int:
def FAS_ServoAlarmReset(iBdID):
    return FAS_ServoAlarmReset_Original(ctypes.c_int(iBdID))


# def FAS_StepAlarmReset(iBdID: int, bReset: bool) -> int:
def FAS_StepAlarmReset(iBdID, bReset):
    return FAS_StepAlarmReset_Original(ctypes.c_int(iBdID), ctypes.c_int(bReset))


# def FAS_BrakeSet(iBdID: int, bSet: bool) -> tuple[int, int]:
def FAS_BrakeSet(iBdID, bSet):
    nResult = ctypes.c_int()
    result = FAS_BrakeSet_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(bSet),
        ctypes.byref(nResult)
    )
    if result == 0:
        return result, nResult.value
    else:
        return result, None


# ------------------------------------------------------------------------------
# 					Read Status and Position
# ------------------------------------------------------------------------------


# def FAS_GetAxisStatus(iBdID: int) -> tuple[int, int]:
def FAS_GetAxisStatus(iBdID):
    dwAxisStatus = ctypes.c_uint()
    result = FAS_GetAxisStatus_Original(ctypes.c_int(iBdID), ctypes.byref(dwAxisStatus))
    if result == 0:
        return result, dwAxisStatus.value
    else:
        return result, None


# def FAS_GetIOAxisStatus(iBdID: int) -> tuple[int, int, int, int]:
def FAS_GetIOAxisStatus(iBdID):
    dwInStatus = ctypes.c_uint()
    dwOutStatus = ctypes.c_uint()
    dwAxisStatus = ctypes.c_uint()

    result = FAS_GetIOAxisStatus_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(dwInStatus),
        ctypes.byref(dwOutStatus),
        ctypes.byref(dwAxisStatus)
    )
    if result == 0:
        return result, dwInStatus.value, dwOutStatus.value, dwAxisStatus.value
    else:
        return result, None, None, None


# def FAS_GetMotionStatus(iBdID: int) -> tuple[int, int, int, int, int, int]:
def FAS_GetMotionStatus(iBdID):
    lCmdPos = ctypes.c_int()
    lActPos = ctypes.c_int()
    lPosErr = ctypes.c_int()
    lActVel = ctypes.c_int()
    wPosItemNo = ctypes.c_ushort()

    result = FAS_GetMotionStatus_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(lCmdPos),
        ctypes.byref(lActPos),
        ctypes.byref(lPosErr),
        ctypes.byref(lActVel),
        ctypes.byref(wPosItemNo)
    )
    if result == 0:
        return (
            result,
            lCmdPos.value,
            lActPos.value,
            lPosErr.value,
            lActVel.value,
            wPosItemNo.value
        )
    else:
        return result, None, None, None, None, None


# def FAS_GetAllStatus(iBdID: int) -> tuple[int, int, int, int, int, int, int, int, int]:
def FAS_GetAllStatus(iBdID):
    dwInStatus = ctypes.c_uint()
    dwOutStatus = ctypes.c_uint()
    dwAxisStatus = ctypes.c_uint()
    lCmdPos = ctypes.c_int()
    lActPos = ctypes.c_int()
    lPosErr = ctypes.c_int()
    lActVel = ctypes.c_int()
    wPosItemNo = ctypes.c_ushort()

    result = FAS_GetAllStatus_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(dwInStatus),
        ctypes.byref(dwOutStatus),
        ctypes.byref(dwAxisStatus),
        ctypes.byref(lCmdPos),
        ctypes.byref(lActPos),
        ctypes.byref(lPosErr),
        ctypes.byref(lActVel),
        ctypes.byref(wPosItemNo)
    )
    if result == 0:
        return (
            result,
            dwInStatus.value,
            dwOutStatus.value,
            dwAxisStatus.value,
            lCmdPos.value,
            lActPos.value,
            lPosErr.value,
            lActVel.value,
            wPosItemNo.value
        )
    else:
        return result, None, None, None, None, None, None, None, None


# def FAS_GetAllStatusEx(iBdID: int, pTypes: list) -> tuple[int, list]:
def FAS_GetAllStatusEx(iBdID, pTypes):
    pTypesarray = (ctypes.c_ubyte * len(pTypes))(*pTypes)
    pDatas = (ctypes.c_int * 12)()
    result = FAS_GetAllStatusEx_Original(
        ctypes.c_int(iBdID), pTypesarray, ctypes.cast(pDatas, ctypes.POINTER(ctypes.c_int))
    )

    if result == 0:
        return result, list(pDatas)
    else:
        return result, None


# def FAS_SetCommandPos(iBdID: int, lCmdPos: int) -> int:
def FAS_SetCommandPos(iBdID, lCmdPos):
    return FAS_SetCommandPos_Original(ctypes.c_int(iBdID), ctypes.c_int(lCmdPos))


# def FAS_SetActualPos(iBdID: int, lActPos: int) -> int:
def FAS_SetActualPos(iBdID, lActPos):
    return FAS_SetActualPos_Original(ctypes.c_int(iBdID), ctypes.c_int(lActPos))


# def FAS_ClearPosition(iBdID: int) -> int:
def FAS_ClearPosition(iBdID):
    return FAS_ClearPosition_Original(ctypes.c_int(iBdID))


# def FAS_GetCommandPos(iBdID: int) -> tuple[int, int]:
def FAS_GetCommandPos(iBdID):
    lCmdPos = ctypes.c_int()
    result = FAS_GetCommandPos_Original(ctypes.c_int(iBdID), ctypes.byref(lCmdPos))
    if result == 0:
        return result, lCmdPos.value
    else:
        return result, None


# def FAS_GetActualPos(iBdID: int) -> tuple[int, int]:
def FAS_GetActualPos(iBdID):
    lActPos = ctypes.c_int()
    result = FAS_GetActualPos_Original(ctypes.c_int(iBdID), ctypes.byref(lActPos))
    if result == 0:
        return result, lActPos.value
    else:
        return result, None


# def FAS_GetPosError(iBdID: int) -> tuple[int, int]:
def FAS_GetPosError(iBdID):
    lPosErr = ctypes.c_int()
    result = FAS_GetPosError_Original(ctypes.c_int(iBdID), ctypes.byref(lPosErr))
    if result == 0:
        return result, lPosErr.value
    else:
        return result, None


# def FAS_GetActualVel(iBdID: int) -> tuple[int, int]:
def FAS_GetActualVel(iBdID):
    lActVel = ctypes.c_int()
    result = FAS_GetActualVel_Original(ctypes.c_int(iBdID), ctypes.byref(lActVel))
    if result == 0:
        return result, lActVel.value
    else:
        return result, None


# def FAS_GetAlarmType(iBdID: int) -> tuple[int, int]:
def FAS_GetAlarmType(iBdID):
    nAlarmType = ctypes.c_ubyte()
    result = FAS_GetAlarmType_Original(ctypes.c_int(iBdID), ctypes.byref(nAlarmType))
    if result == 0:
        return result, nAlarmType.value
    else:
        return result, None


# ------------------------------------------------------------------
# 					Motion Functions.
# ------------------------------------------------------------------
# def FAS_MoveStop(iBdID: int) -> int:
def FAS_MoveStop(iBdID):
    return FAS_MoveStop_Original(ctypes.c_int(iBdID))


# def FAS_EmergencyStop(iBdID: int) -> int:
def FAS_EmergencyStop(iBdID):
    return FAS_EmergencyStop_Original(ctypes.c_int(iBdID))


# def FAS_MovePause(iBdID: int, bPause: bool) -> int:
def FAS_MovePause(iBdID, bPause):
    return FAS_MovePause_Original(ctypes.c_int(iBdID), ctypes.c_int(bPause))


# def FAS_MoveOriginSingleAxis(iBdID: int) -> int:
def FAS_MoveOriginSingleAxis(iBdID):
    return FAS_MoveOriginSingleAxis_Original(ctypes.c_int(iBdID))


# def FAS_MoveSingleAxisAbsPos(iBdID: int, lAbsPos: int, lVelocity: int) -> int:
def FAS_MoveSingleAxisAbsPos(iBdID, lAbsPos, lVelocity):
    return FAS_MoveSingleAxisAbsPos_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(lAbsPos),
        ctypes.c_uint(lVelocity)
    )


# def FAS_MoveSingleAxisIncPos(iBdID: int, lIncPos: int, lVelocity: int) -> int:
def FAS_MoveSingleAxisIncPos(iBdID, lIncPos, lVelocity):
    return FAS_MoveSingleAxisIncPos_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(lIncPos),
        ctypes.c_uint(lVelocity)
    )


# def FAS_MoveToLimit(iBdID: int, lVelocity: int, iLimitDir: int) -> int:
def FAS_MoveToLimit(iBdID, lVelocity, iLimitDir):
    return FAS_MoveToLimit_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(lVelocity),
        ctypes.c_int(iLimitDir)
    )


# def FAS_MoveVelocity(iBdID: int, lVelocity: int, iVelDir: int) -> int:
def FAS_MoveVelocity(iBdID, lVelocity, iVelDir):
    return FAS_MoveVelocity_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(lVelocity),
        ctypes.c_int(iVelDir)
    )


# def FAS_PositionAbsOverride(iBdID: int, lOverridePos: int) -> int:
def FAS_PositionAbsOverride(iBdID, lOverridePos):
    return FAS_PositionAbsOverride_Original(
        ctypes.c_int(iBdID), ctypes.c_int(lOverridePos)
    )


# def FAS_PositionIncOverride(iBdID: int, lOverridePos: int) -> int:
def FAS_PositionIncOverride(iBdID, lOverridePos):
    return FAS_PositionIncOverride_Original(
        ctypes.c_int(iBdID), ctypes.c_int(lOverridePos)
    )


# def FAS_VelocityOverride(iBdID: int, lVelocity: int) -> int:
def FAS_VelocityOverride(iBdID, lVelocity):
    return FAS_VelocityOverride_Original(ctypes.c_int(iBdID), ctypes.c_uint(lVelocity))


# def FAS_MoveLinearAbsPos(nNoOfBds: int, iBdID: list, lplAbsPos: list, lFeedrate: int, wAccelTime: int) -> int:
def FAS_MoveLinearAbsPos(
    nNoOfBds,
    iBdID,
    lplAbsPos,
    lFeedrate,
    wAccelTime
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplAbsPosArray = (ctypes.c_int * nNoOfBds)(*lplAbsPos)
    result = FAS_MoveLinearAbsPos_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplAbsPosArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime)
    )
    return result

#def FAS_MoveLinearIncPos(nNoOfBds: int, iBdID: list, lplIncPos: list, lFeedrate: int, wAccelTime: int) -> int:
def FAS_MoveLinearIncPos(
    nNoOfBds,
    iBdID,
    lplIncPos,
    lFeedrate,
    wAccelTime
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplIncPosArray = (ctypes.c_int * nNoOfBds)(*lplIncPos)
    result = FAS_MoveLinearIncPos_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplIncPosArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime)
    )
    return result

# def FAS_MoveLinearAbsPos2(nNoOfBds: int, iBdID: list, lplAbsPos: list, lFeedrate: int, wAccelTime: int) -> int:
def FAS_MoveLinearAbsPos2(
    nNoOfBds,
    iBdID,
    lplAbsPos,
    lFeedrate,
    wAccelTime
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplAbsPosArray = (ctypes.c_int * nNoOfBds)(*lplAbsPos)
    result = FAS_MoveLinearAbsPos2_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplAbsPosArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime)
    )
    return result

# def FAS_MoveLinearIncPos2(nNoOfBds: int, iBdID: list, lplIncPos: list, lFeedrate: int, wAccelTime: int) -> int:

def FAS_MoveLinearIncPos2(
    nNoOfBds,
    iBdID,
    lplIncPos,
    lFeedrate,
    wAccelTime
):

    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplIncPosArray = (ctypes.c_int * nNoOfBds)(*lplIncPos)
    result = FAS_MoveLinearIncPos2_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplIncPosArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime)
    )
    return result

# def FAS_MoveCircleAbsPos1(nNoOfBds: int, iBdID: list, lplCirEndAbs: list, lplCirCenterAbs: list, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleAbsPos1(
    nNoOfBds,
    iBdID,
    lplCirEndAbs,
    lplCirCenterAbs,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirEndAbsArray = (ctypes.c_int * nNoOfBds)(*lplCirEndAbs)
    lplCirCenterAbsArray = (ctypes.c_int * nNoOfBds)(*lplCirCenterAbs)
    result = FAS_MoveCircleAbsPos1_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirEndAbsArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirCenterAbsArray,ctypes.POINTER(ctypes.c_int)),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_MoveCircleIncPos1(nNoOfBds: int, iBdID: list, lplCirEndInc: list, lplCirCenterInc: list, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleIncPos1(
    nNoOfBds,
    iBdID,
    lplCirEndInc,
    lplCirCenterInc,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirEndIncArray = (ctypes.c_int * nNoOfBds)(*lplCirEndInc)
    lplCirCenterIncArray = (ctypes.c_int * nNoOfBds)(*lplCirCenterInc)
    result = FAS_MoveCircleIncPos1_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirEndIncArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirCenterIncArray,ctypes.POINTER(ctypes.c_int)),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_MoveCircleAbsPos2(nNoOfBds: int, iBdID: list, lplCirEndAbs: list, lRadius: int, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleAbsPos2(
    nNoOfBds,
    iBdID,
    lplCirEndAbs,
    lRadius,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirEndAbsArray = (ctypes.c_int * nNoOfBds)(*lplCirEndAbs)
    result = FAS_MoveCircleAbsPos2_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirEndAbsArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lRadius),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_MoveCircleIncPos2(nNoOfBds: int, iBdID: list, lplCirEndInc: list, lRadius: int, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleIncPos2(
    nNoOfBds,
    iBdID,
    lplCirEndInc,
    lRadius,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirEndIncArray = (ctypes.c_int * nNoOfBds)(*lplCirEndInc)
    result = FAS_MoveCircleIncPos2_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirEndIncArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(lRadius),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_MoveCircleAbsPos3(nNoOfBds: int, iBdID: list, lplCirCenterAbs: list, nAngle: int, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleAbsPos3(
    nNoOfBds,
    iBdID,
    lplCirCenterAbs,
    nAngle,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirCenterAbsArray = (ctypes.c_int * nNoOfBds)(*lplCirCenterAbs)
    result = FAS_MoveCircleAbsPos3_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirCenterAbsArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(nAngle),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_MoveCircleIncPos3(nNoOfBds: int, iBdID: list, lplCirCenterInc: list, nAngle: int, iDirection: int, lFeedrate: int, wAccelTime: int, bSCurve: int) -> int:
def FAS_MoveCircleIncPos3(
    nNoOfBds,
    iBdID,
    lplCirCenterInc,
    nAngle,
    iDirection,
    lFeedrate,
    wAccelTime,
    bSCurve
):
    iBdIDArray = (ctypes.c_int * nNoOfBds)(*iBdID)
    lplCirCenterIncArray = (ctypes.c_int * nNoOfBds)(*lplCirCenterInc)
    result = FAS_MoveCircleIncPos3_Original(
        ctypes.c_ubyte(nNoOfBds),
        ctypes.cast(iBdIDArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.cast(lplCirCenterIncArray, ctypes.POINTER(ctypes.c_int)),
        ctypes.c_uint(nAngle),
        ctypes.c_int(iDirection),
        ctypes.c_uint(lFeedrate),
        ctypes.c_ushort(wAccelTime),
        ctypes.c_int(bSCurve)
    )
    return result

# def FAS_TriggerOutput_RunA(iBdID: int, bStartTrigger: bool, lStartPos: int, dwPeriod: int, dwPulseTime: int) -> int:
def FAS_TriggerOutput_RunA(
    iBdID,
    bStartTrigger,
    lStartPos,
    dwPeriod,
    dwPulseTime
):
    return FAS_TriggerOutput_RunA_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(bStartTrigger),
        ctypes.c_int(lStartPos),
        ctypes.c_uint(dwPeriod),
        ctypes.c_uint(dwPulseTime)
    )


# def FAS_TriggerOutput_Status(iBdID: int) -> tuple[int, int]:
def FAS_TriggerOutput_Status(iBdID):
    bTriggerStatus = ctypes.c_ubyte()
    result = FAS_TriggerOutput_Status_Original(
        ctypes.c_int(iBdID), ctypes.byref(bTriggerStatus)
    )
    if result == 0:
        return result, bTriggerStatus.value
    else:
        return result, None

# def FAS_SetTriggerOutputEx(iBdID: int, nOutputNo: int, bRun: int, wOnTime: int, nTriggerCount: int, arrTriggerPosition: list) -> int:
def FAS_SetTriggerOutputEx(
    iBdID,
    nOutputNo,
    bRun,
    wOnTime,
    nTriggerCount,
    arrTriggerPosition
):
    arrType = (ctypes.c_int * nTriggerCount)(*arrTriggerPosition)
    result = FAS_SetTriggerOutputEx_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(nOutputNo),
        ctypes.c_ubyte(bRun),
        ctypes.c_ushort(wOnTime),
        ctypes.c_ubyte(nTriggerCount),
        ctypes.cast(arrType, ctypes.POINTER(ctypes.c_int))
    )

    return result

# def FAS_GetTriggerOutputEx(iBdID: int, nOutputNo: int) -> tuple[int, int, int, int, List]:
def FAS_GetTriggerOutputEx(
    iBdID, nOutputNo
):
    bRun = ctypes.c_ubyte()
    wOnTime = ctypes.c_ushort()
    nTriggerCount = ctypes.c_ubyte()
    arrTriggerPosition = ctypes.c_int()
    result = FAS_GetTriggerOutputEx_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(nOutputNo),
        ctypes.byref(bRun),
        ctypes.byref(wOnTime),
        ctypes.byref(nTriggerCount),
        ctypes.byref(arrTriggerPosition)
    )
    if result == 0:
        return (
            result,
            bRun.value,
            wOnTime.value,
            nTriggerCount.value,
            list(arrTriggerPosition)
        )
    else:
        return result, None, None, None, None, None

# def FAS_MovePush(iBdID: int, dwStartSpd: int, dwMoveSpd: int, lPosition: int, wAccel: int, wDecel: int, wPushRate: int, dwPushSpd: int, lEndPosition: int, wPushMode: int) -> int:
def FAS_MovePush(
    iBdID,
    dwStartSpd,
    dwMoveSpd,
    lPosition,
    wAccel,
    wDecel,
    wPushRate,
    dwPushSpd,
    lEndPosition,
    wPushMode
):
    return FAS_MovePush_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(dwStartSpd),
        ctypes.c_uint(dwMoveSpd),
        ctypes.c_int(lPosition),
        ctypes.c_ushort(wAccel),
        ctypes.c_ushort(wDecel),
        ctypes.c_ushort(wPushRate),
        ctypes.c_uint(dwPushSpd),
        ctypes.c_int(lEndPosition),
        ctypes.c_ushort(wPushMode)
    )


# def FAS_GetPushStatus(iBdID: int) -> tuple[int, int]:
def FAS_GetPushStatus(iBdID):
    nPushStatus = ctypes.c_ubyte()
    result = FAS_GetPushStatus_Original(ctypes.c_int(iBdID), ctypes.byref(nPushStatus))
    if result == 0:
        return result, nPushStatus.value
    else:
        return result, None


# ------------------------------------------------------------------
# 					Ex-Motion Functions.
# ------------------------------------------------------------------
#def FAS_MoveSingleAxisAbsPosEx(iBdID: int, lAbsPos: int, lVelocity: int, lpExOption: MOTION_OPTION_EX) -> int:
def FAS_MoveSingleAxisAbsPosEx(
    iBdID,
    lAbsPos,
    lVelocity,
    lpExOption
):
    opt = MOTION_OPTION_EX_to_CMOTION_OPTION_EX(lpExOption)
    result = FAS_MoveSingleAxisAbsPosEx_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(lAbsPos),
        ctypes.c_uint(lVelocity),
        ctypes.byref(opt)
    )
    return result

# def FAS_MoveSingleAxisIncPosEx(iBdID: int, lIncPos: int, lVelocity: int, lpExOption: MOTION_OPTION_EX) -> int:
def FAS_MoveSingleAxisIncPosEx(
    iBdID,
    lIncPos,
    lVelocity,
    lpExOption
):
    opt = MOTION_OPTION_EX_to_CMOTION_OPTION_EX(lpExOption)
    result = FAS_MoveSingleAxisIncPosEx_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(lIncPos),
        ctypes.c_uint(lVelocity),
        ctypes.byref(opt)
    )
    return result

# def FAS_MoveVelocityEx(iBdID: int, lVelocity: int, iVelDir: int, lpExOption: VELOCITY_OPTION_EX) -> int:
def FAS_MoveVelocityEx(
    iBdID,
    lVelocity,
    iVelDir,
    lpExOption
):
    opt = VELOCITY_OPTION_EX_to_CVELOCITY_OPTION_EX(lpExOption)
    result = FAS_MoveVelocityEx_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(lVelocity),
        ctypes.c_int(iVelDir),
        ctypes.byref(opt)
    )
    return result


# ------------------------------------------------------------------
# 					Position Table Functions.
# ------------------------------------------------------------------


# def FAS_PosTableReadItem(iBdID: int, wItemNo: int) -> tuple[int, List]:
def FAS_PosTableReadItem(iBdID, wItemNo):
    Item = CITEM_NODE()
    result = FAS_PosTableReadItem_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ushort(wItemNo),
        byref(Item)
    )
    if result == 0:
        lpItem = CITEM_NODE_to_ITEM_NODE(Item)
        return result, lpItem
    else:
        return result, None


# def FAS_PosTableWriteItem(iBdID: int, wItemNo: int, lpItem: ITEM_NODE) -> int:
def FAS_PosTableWriteItem(iBdID, wItemNo, lpItem):
    Item = ITEM_NODE_to_CITEM_NODE(lpItem)
    result = FAS_PosTableWriteItem_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ushort(wItemNo),
        ctypes.byref(Item)
    )

    return result


# def FAS_PosTableWriteROM(iBdID: int) -> int:
def FAS_PosTableWriteROM(iBdID):
    return FAS_PosTableWriteROM_Original(ctypes.c_int(iBdID))

# def FAS_PosTableReadROM(iBdID: int) -> int:
def FAS_PosTableReadROM(iBdID):
    return FAS_PosTableReadROM_Original(ctypes.c_int(iBdID))


# def FAS_PosTableRunItem(iBdID: int, wItemNo: int) -> int:
def FAS_PosTableRunItem(iBdID, wItemNo):
    return FAS_PosTableRunItem_Original(ctypes.c_int(iBdID), ctypes.c_ushort(wItemNo))


# def FAS_PosTableReadOneItem(iBdID: int, wItemNo: int, wOffset: int) -> tuple[int, int]:
def FAS_PosTableReadOneItem(iBdID, wItemNo, wOffset):
    lPosItemVal = ctypes.c_int()
    result = FAS_PosTableReadOneItem_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ushort(wItemNo),
        ctypes.c_ushort(wOffset),
        ctypes.byref(lPosItemVal)
    )
    if result == 0:
        return result, lPosItemVal.value
    else:
        return result, None

# def FAS_PosTableWriteOneItem(iBdID: int, wItemNo: int, wOffset: int, lPosItemVal: int) -> int:
def FAS_PosTableWriteOneItem(
    iBdID, wItemNo, wOffset, lPosItemVal
):
    return FAS_PosTableWriteOneItem_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ushort(wItemNo),
        ctypes.c_ushort(wOffset),
        ctypes.c_int(lPosItemVal)
    )


# def FAS_PosTableSingleRunItem(iBdID: int, bNextMove: int, wItemNo: int) -> int:
def FAS_PosTableSingleRunItem(iBdID, bNextMove, wItemNo):
    return FAS_PosTableSingleRunItem_Original(
        ctypes.c_int(iBdID),
        ctypes.c_int(bNextMove),
        ctypes.c_ushort(wItemNo)
    )


# ------------------------------------------------------------------
# 					Gap Control Functions.
# ------------------------------------------------------------------
# def FAS_GapControlEnable(iBdID: int, wItemNo: int, lGapCompSpeed: int, lGapAccTime: int, lGapDecTime: int, lGapStartSpeed: int) -> int:
def FAS_GapControlEnable(
    iBdID,
    wItemNo,
    lGapCompSpeed,
    lGapAccTime,
    lGapDecTime,
    lGapStartSpeed
):
    return FAS_GapControlEnable_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ushort(wItemNo),
        ctypes.c_int(lGapCompSpeed),
        ctypes.c_int(lGapAccTime),
        ctypes.c_int(lGapDecTime),
        ctypes.c_int(lGapStartSpeed)
    )


# def FAS_GapControlDisable(iBdID: int) -> int:
def FAS_GapControlDisable(iBdID):
    return FAS_GapControlDisable_Original(ctypes.c_int(iBdID))


# def FAS_IsGapControlEnable(iBdID: int) -> tuple[int, int, int]:
def FAS_IsGapControlEnable(iBdID):
    bIsEnable = ctypes.c_int()
    wCurrentItemNo = ctypes.c_ushort()
    result = FAS_IsGapControlEnable_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(bIsEnable),
        ctypes.byref(wCurrentItemNo),
    )
    if result == 0:
        return result, bIsEnable.value, wCurrentItemNo.value
    else:
        return result, None, None


# def FAS_GapControlGetADCValue(iBdID: int) -> tuple[int, int]:
def FAS_GapControlGetADCValue(iBdID):
    lADCValue = ctypes.c_int()
    result = FAS_GapControlGetADCValue_Original(
        ctypes.c_int(iBdID), ctypes.byref(lADCValue)
    )
    if result == 0:
        return result, lADCValue.value
    else:
        return result, None

# def FAS_GapOneResultMonitor(iBdID: int) -> tuple[int, int, int, int, int, int, int, int]:
def FAS_GapOneResultMonitor(iBdID):
    bUpdated = ctypes.c_ubyte()
    iIndex = ctypes.c_int()
    lGapValue = ctypes.c_int()
    lCmdPos = ctypes.c_int()
    lActPos = ctypes.c_int()
    lCompValue = ctypes.c_int()
    lReserved = ctypes.c_int()
    result = FAS_GapOneResultMonitor_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(bUpdated),
        ctypes.byref(iIndex),
        ctypes.byref(lGapValue),
        ctypes.byref(lCmdPos),
        ctypes.byref(lActPos),
        ctypes.byref(lCompValue),
        ctypes.byref(lReserved),
    )
    if result == 0:
        return (
            result,
            bUpdated.value,
            iIndex.value,
            lGapValue.value,
            lCmdPos.value,
            lActPos.value,
            lCompValue.value,
            lReserved.value
        )
    else:
        return result, None, None, None, None, None, None, None


# ------------------------------------------------------------------
# 					Alarm Type History Functions.
# ------------------------------------------------------------------
# def FAS_GetAlarmLogs(iBdID: int) -> tuple[int, List]:
def FAS_GetAlarmLogs(iBdID):
    AlarmLog = CALARM_LOG()
    result = FAS_GetAlarmLogs_Original(ctypes.c_int(iBdID), ctypes.byref(AlarmLog))
    if result == 0:
        pAlarmLog = CALARM_LOG_to_ALARM_LOG(AlarmLog)
        return result, pAlarmLog
    else:
        return result, None


#def FAS_ResetAlarmLogs(iBdID: int) -> int:
def FAS_ResetAlarmLogs(iBdID):
    return FAS_ResetAlarmLogs_Original(ctypes.c_int(iBdID))


# ------------------------------------------------------------------
# 					I/O Module Functions.
# ------------------------------------------------------------------
# def FAS_GetInput(iBdID: int) -> tuple[int, int, int]:
def FAS_GetInput(iBdID):
    uInput = ctypes.c_uint()
    uLatch = ctypes.c_uint()
    result = FAS_GetInput_Original(
        ctypes.c_int(iBdID),
        ctypes.byref(uInput),
        ctypes.byref(uLatch)
    )
    if result == 0:
        return result, uInput.value, uLatch.value
    else:
        return result, None, None


# def FAS_ClearLatch(iBdID: int, uLatchMask: int) -> int:
def FAS_ClearLatch(iBdID, uLatchMask):
    return FAS_ClearLatch_Original(ctypes.c_int(iBdID), ctypes.c_uint(uLatchMask))


#def FAS_GetLatchCount(iBdID: int, iInputNo: int) -> tuple[int, int]:
def FAS_GetLatchCount(iBdID, iInputNo):
    uCount = ctypes.c_uint()
    result = FAS_GetLatchCount_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(iInputNo),
        ctypes.byref(uCount)
    )
    if result == 0:
        return result, uCount.value
    else:
        return result, None


# def FAS_GetLatchCountAll(iBdID: int) -> tuple[int, list]:
def FAS_GetLatchCountAll(iBdID):
    ppuAllCount = (ctypes.c_uint * 16)()
    result = FAS_GetLatchCountAll_Original(
        ctypes.c_int(iBdID), ctypes.byref(ppuAllCount)
    )
    if result == 0:
        return result, list(ppuAllCount)
    else:
        return result, None


# def FAS_GetLatchCountAll32(iBdID: int) -> tuple[int, int]:
def FAS_GetLatchCountAll32(iBdID):
    ppuAllCount = (ctypes.c_uint * 32)()
    result = FAS_GetLatchCountAll32_Original(
        ctypes.c_int(iBdID), ctypes.byref(ppuAllCount)
    )
    if result == 0:
        return result, list(ppuAllCount)
    else:
        return result, None


# def FAS_ClearLatchCount(iBdID: int, uInputMask: int) -> int:
def FAS_ClearLatchCount(iBdID, uInputMask):
    return FAS_ClearLatchCount_Original(ctypes.c_int(iBdID), ctypes.c_uint(uInputMask))


# def FAS_GetOutput(iBdID: int) -> tuple[int, int, int]:
def FAS_GetOutput(iBdID):
    uOutput = ctypes.c_uint()
    uStatus = ctypes.c_uint()
    result = FAS_GetOutput_Original(
        ctypes.c_int(iBdID), ctypes.byref(uOutput), ctypes.byref(uStatus)
    )
    if result == 0:
        return result, uOutput.value, uStatus.value
    else:
        return result, None, None


# def FAS_SetOutput(iBdID: int, uSetMask: int, uClearMask: int) -> int:
def FAS_SetOutput(iBdID, uSetMask, uClearMask):
    return FAS_SetOutput_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(uSetMask),
        ctypes.c_uint(uClearMask)
    )


# def FAS_SetTrigger(iBdID: int, uOutputNo: int, pTrigger: TRIGGER_INFO) -> int:
def FAS_SetTrigger(iBdID, uOutputNo, pTrigger):
    Trigger = TRIGGER_INFO_to_CTRIGGER_INFO(pTrigger)
    result = FAS_SetTrigger_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(uOutputNo),
        ctypes.byref(Trigger)
    )
    return result


# def FAS_SetRunStop(iBdID: int, uRunMask: int, uStopMask: int) -> int:
def FAS_SetRunStop(iBdID, uRunMask, uStopMask):
    return FAS_SetRunStop_Original(
        ctypes.c_int(iBdID),
        ctypes.c_uint(uRunMask),
        ctypes.c_uint(uStopMask)
    )


# def FAS_GetTriggerCount(iBdID: int, uOutputNo: int) -> tuple[int, int]:
def FAS_GetTriggerCount(iBdID, uOutputNo):
    uCount = ctypes.c_uint()
    result = FAS_GetTriggerCount_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(uOutputNo),
        ctypes.byref(uCount)
    )
    if result == 0:
        return result, uCount.value
    else:
        return result, None


# def FAS_GetIOLevel(iBdID: int) -> tuple[int, int]:
def FAS_GetIOLevel(iBdID):
    uIOLevel = ctypes.c_uint()
    result = FAS_GetIOLevel_Original(ctypes.c_int(iBdID), ctypes.byref(uIOLevel))
    if result == 0:
        return result, uIOLevel.value
    else:
        return result, None


# def FAS_SetIOLevel(iBdID: int, uIOLevel: int) -> int:
def FAS_SetIOLevel(iBdID, uIOLevel):
    return FAS_SetIOLevel_Original(ctypes.c_int(iBdID), ctypes.c_uint(uIOLevel))


# def FAS_LoadIOLevel(iBdID: int) -> int:
def FAS_LoadIOLevel(iBdID):
    return FAS_LoadIOLevel_Original(ctypes.c_int(iBdID))


# def FAS_SaveIOLevel(iBdID: int) -> int:
def FAS_SaveIOLevel(iBdID):
    return FAS_SaveIOLevel_Original(ctypes.c_int(iBdID))


# def FAS_GetInputFilter(iBdID: int) -> tuple[int, int]:
def FAS_GetInputFilter(iBdID):
    filter = ctypes.c_ushort()
    result = FAS_GetInputFilter_Original(ctypes.c_int(iBdID), ctypes.byref(filter))
    if result == 0:
        return result, filter.value
    else:
        return result, None


# def FAS_SetInputFilter(iBdID: int, filter: int) -> int:
def FAS_SetInputFilter(iBdID, filter):
    return FAS_SetInputFilter_Original(ctypes.c_int(iBdID), ctypes.c_ushort(filter))


# def FAS_GetIODirection(iBdID: int) -> tuple[int, int]:
def FAS_GetIODirection(iBdID):
    direction = ctypes.c_uint()
    result = FAS_GetIODirection_Original(ctypes.c_int(iBdID), ctypes.byref(direction))
    if result == 0:
        return result, direction.value
    else:
        return result, None


# def FAS_SetIODirection(iBdID: int, direction: int) -> int:
def FAS_SetIODirection(iBdID, direction):
    return FAS_SetIODirection_Original(ctypes.c_int(iBdID), ctypes.c_uint(direction))


# ------------------------------------------------------------------
# 					Ezi-IO AD Functions
# ------------------------------------------------------------------
# def FAS_SetADConfig(iBdID: int, channel: int, type: int, value: int) -> tuple[int,int]:
def FAS_SetADConfig(iBdID, channel, type, value):
    recv = ctypes.c_int()
    result = FAS_SetADConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.c_int(value),
        ctypes.byref(recv)
    )
    if result == 0:
        return result, recv.value
    else:
        return result, None



# def FAS_GetADConfig(iBdID: int, channel: int, type: int) -> tuple[int, int]:
def FAS_GetADConfig(iBdID, channel, type):
    value = ctypes.c_int()
    result = FAS_GetADConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.byref(value)
    )
    if result == 0:
        return result, value.value
    else:
        return result, None


# def FAS_LoadADConfig(iBdID: int) -> int:
def FAS_LoadADConfig(iBdID):
    return FAS_LoadADConfig_Original(ctypes.c_int(iBdID))


# def FAS_SaveADConfig(iBdID: int) -> int:
def FAS_SaveADConfig(iBdID):
    return FAS_SaveADConfig_Original(ctypes.c_int(iBdID))


# def FAS_ReadADValue(iBdID: int, channel: int) -> tuple[int, int]:
def FAS_ReadADValue(iBdID, channel):
    advalue = ctypes.c_short()
    result = FAS_ReadADValue_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.byref(advalue)
    )
    if result == 0:
        return result, advalue.value
    else:
        return result, None


# def FAS_ReadADAllValue(iBdID: int, offset: int) -> tuple[int, list]:
def FAS_ReadADAllValue(iBdID, offset):
    adbuffer = (ctypes.c_short * 8)()
    result = FAS_ReadADAllValue_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(offset),
        adbuffer
    )
    if result == 0:
        return result, list(adbuffer)
    else:
        return result, None


# def FAS_GetAllADResult(iBdID: int) -> tuple[int, list]:
def FAS_GetAllADResult(iBdID):
    result_array = AD_RESULT()
    result = FAS_GetAllADResult_Original(
        ctypes.c_int(iBdID), ctypes.byref(result_array)
    )
    if result == 0:
        return result, list(result_array.elements)
    else:
        return result, None

# def FAS_GetADResult(iBdID: int, channel: int) -> tuple[int, float]:
def FAS_GetADResult(iBdID, channel):  
    adresult = ctypes.c_float()
    result = FAS_GetADResult_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.byref(adresult),
    )
    if result == 0:
        return result, adresult.value
    else:
        return result, None


# def FAS_SetADRange(iBdID: int, channel: int, range: int) -> int:
def FAS_SetADRange(iBdID, channel, range):
    return FAS_SetADRange_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(range)
    )


# ------------------------------------------------------------------
# 			Ezi-IO DA Functions
# ------------------------------------------------------------------
# def FAS_SetDACConfig(iBdID: int, channel: int, type: int, data: int) -> tuple[int,int]:
def FAS_SetDACConfig(iBdID, channel, type, data):
    recv = ctypes.c_int()
    result = FAS_SetDACConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.c_int(data),
        ctypes.byref(recv)
    )
    if result == 0:
        return result, recv.value
    else:
        return result, None

# def FAS_GetDACConfig(iBdID: int, channel: int, type: int) -> tuple[int, int]:
def FAS_GetDACConfig(iBdID, channel, type):
    data = ctypes.c_int()
    result = FAS_GetDACConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.byref(data)
    )
    if result == 0:
        return result, data.value
    else:
        return result, None


# def FAS_LoadDACConfig(iBdID: int) -> int:
def FAS_LoadDACConfig(iBdID):
    return FAS_LoadDACConfig_Original(ctypes.c_int(iBdID))


# def FAS_SaveDACConfig(iBdID: int) -> int:
def FAS_SaveDACConfig(iBdID):
    return FAS_SaveDACConfig_Original(ctypes.c_int(iBdID))


# def FAS_SetDACValue(iBdID: int, channel: int, bEnable: int, value: int) -> int:
def FAS_SetDACValue(iBdID, channel, bEnable, value):
    return FAS_SetDACValue_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(bEnable),
        ctypes.c_int(value)
    )


# def FAS_GetDACValue(iBdID: int, channel: int) -> tuple[int, int, int]:
def FAS_GetDACValue(iBdID, channel):
    bEnable = ctypes.c_int()
    value = ctypes.c_int()
    result = FAS_GetDACValue_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.byref(bEnable),
        ctypes.byref(value)
    )
    if result == 0:
        return result, bEnable.value, value.value
    else:
        return result, None, None


# ------------------------------------------------------------------
# 			Ezi-IO Counter Functions
# ------------------------------------------------------------------


# def FAS_CounterCommand(iBdID: int, channel: int, cmd: int, data: int) -> int:
def FAS_CounterCommand(iBdID, channel, cmd, data):
    return FAS_CounterCommand_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(cmd),
        ctypes.c_int(data)
    )


# def FAS_GetCounterValue(iBdID: int, channel: int, type: int) -> tuple[int, int]:
def FAS_GetCounterValue(iBdID, channel, type):
    value = ctypes.c_int()
    result = FAS_GetCounterValue_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.byref(value)
    )
    if result == 0:
        return result, value.value
    else:
        return result, None


# def FAS_GetCounterStatus(iBdID: int) -> tuple[int, int]:
def FAS_GetCounterStatus(iBdID):
    dwStatus = ctypes.c_uint()
    result = FAS_GetCounterStatus_Original(ctypes.c_int(iBdID), ctypes.byref(dwStatus))
    if result == 0:
        return result, dwStatus.value
    else:
        return result, None

# def FAS_SetCounterTrigger(iBdID: int, channel: int, bStartTrigger: bool, lStartPos: int, dwPeriod: int, dwPulseTime: int, dwTriggerCnt: int) -> int:
def FAS_SetCounterTrigger(
    iBdID,
    channel,
    bStartTrigger,
    lStartPos,
    dwPeriod,
    dwPulseTime,
    dwTriggerCnt
):
    return FAS_SetCounterTrigger_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(bStartTrigger),
        ctypes.c_int(lStartPos),
        ctypes.c_uint(dwPeriod),
        ctypes.c_uint(dwPulseTime),
        ctypes.c_uint(dwTriggerCnt)
    )

# def FAS_SetCounterConfig(iBdID: int, channel: int, type: int, data: int) -> tuple[int,int]: 
def FAS_SetCounterConfig(iBdID, channel, type, data):
    recv = ctypes.c_int()
    result = FAS_SetCounterConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.c_int(data),
        ctypes.byref(recv)
    )
    if result == 0:
        return result,recv.value
    else:
        return result,None


# def FAS_GetCounterConfig(iBdID: int, channel: int, type: int) -> tuple[int, int]:
def FAS_GetCounterConfig(iBdID, channel, type):
    data = ctypes.c_int()
    result = FAS_GetCounterConfig_Original(
        ctypes.c_int(iBdID),
        ctypes.c_ubyte(channel),
        ctypes.c_int(type),
        ctypes.byref(data)
    )
    if result == 0:
        return result, data.value
    else:
        return result, None


# def FAS_LoadCounterConfig(iBdID: int) -> int:
def FAS_LoadCounterConfig(iBdID):
    return FAS_LoadCounterConfig_Original(ctypes.c_int(iBdID))


# def FAS_SaveCounterConfig(iBdID: int) -> int:
def FAS_SaveCounterConfig(iBdID):
    return FAS_SaveCounterConfig_Original(ctypes.c_int(iBdID))

