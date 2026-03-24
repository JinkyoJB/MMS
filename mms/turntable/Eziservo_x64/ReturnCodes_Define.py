#####################################################
# Version: 1.0.0                                    #
# Last modified date: 25.10.31                      #
# Changes log: First released version               #
#####################################################


# -*- coding: utf-8 -*-
# ------------------------------------------------------------------
#                 Return Code Defines.
# ------------------------------------------------------------------

FMM_OK = 0

FMM_NOT_OPEN = 1
FMM_INVALID_PORT_NUM = 2
FMM_INVALID_SLAVE_NUM = 3

FMC_DISCONNECTED = 5
FMC_TIMEOUT_ERROR = 6
FMC_CRCFAILED_ERROR = 7
FMC_RECVPACKET_ERROR = 8  # PACKET SIZE ERROR

FMM_POSTABLE_ERROR = 9

FMP_FRAMETYPEERROR = 128
FMP_DATAERROR = 129
FMP_PACKETERROR = 130
FMP_RUNFAIL = 133
FMP_RESETFAIL = 134
FMP_SERVOONFAIL1 = 135
FMP_SERVOONFAIL2 = 136
FMP_SERVOONFAIL3 = 137
FMP_SERVOOFF_FAIL = 138
FMP_ROMACCESS = 139

FMP_PACKETCRCERROR = 170

FMM_UNKNOWN_ERROR = 255
