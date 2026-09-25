//
// j2534.h: SAE J2534-1 v04.04 PassThru API definitions.
// Only what a CAN pass-through needs; constants per the published spec.
//
// ABI: every J2534 "unsigned long" is 32 bits, as Windows' unsigned long and
// the ~/.passthru convention on Linux (a 64 bit long there breaks callers).
//
#ifndef J2534_H
#define J2534_H

#include <stdint.h>

#ifdef _WIN32
#define J2534_API __stdcall
#else
#define J2534_API __attribute__((visibility("default")))
#endif

// Protocol IDs
#define J1850VPW     1
#define J1850PWM     2
#define ISO9141      3
#define ISO14230     4
#define CAN          5
#define ISO15765     6
#define SCI_A_ENGINE 7
#define SCI_A_TRANS  8
#define SCI_B_ENGINE 9
#define SCI_B_TRANS  10

// Return codes
#define STATUS_NOERROR            0x00
#define ERR_NOT_SUPPORTED         0x01
#define ERR_INVALID_CHANNEL_ID    0x02
#define ERR_INVALID_PROTOCOL_ID   0x03
#define ERR_NULL_PARAMETER        0x04
#define ERR_INVALID_IOCTL_VALUE   0x05
#define ERR_INVALID_FLAGS         0x06
#define ERR_FAILED                0x07
#define ERR_DEVICE_NOT_CONNECTED  0x08
#define ERR_TIMEOUT               0x09
#define ERR_INVALID_MSG           0x0A
#define ERR_INVALID_TIME_INTERVAL 0x0B
#define ERR_EXCEEDED_LIMIT        0x0C
#define ERR_INVALID_MSG_ID        0x0D
#define ERR_DEVICE_IN_USE         0x0E
#define ERR_INVALID_IOCTL_ID      0x0F
#define ERR_BUFFER_EMPTY          0x10
#define ERR_BUFFER_FULL           0x11
#define ERR_BUFFER_OVERFLOW       0x12
#define ERR_PIN_INVALID           0x13
#define ERR_CHANNEL_IN_USE        0x14
#define ERR_MSG_PROTOCOL_ID       0x15
#define ERR_INVALID_FILTER_ID     0x16
#define ERR_NO_FLOW_CONTROL       0x17
#define ERR_NOT_UNIQUE            0x18
#define ERR_INVALID_BAUDRATE      0x19
#define ERR_INVALID_DEVICE_ID     0x1A

// Connect flags
#define CAN_29BIT_ID_CONNECT 0x00000100
#define CAN_ID_BOTH          0x00000800

// RxStatus bits
#define TX_MSG_TYPE          0x00000001
#define ISO15765_FIRST_FRAME 0x00000002
#define CAN_29BIT_ID         0x00000100 // also a TxFlags bit

// TxFlags bits
#define ISO15765_FRAME_PAD 0x00000040
#define ISO15765_ADDR_TYPE 0x00000080

// Filter types
#define PASS_FILTER         0x01
#define BLOCK_FILTER        0x02
#define FLOW_CONTROL_FILTER 0x03

// Ioctl IDs
#define GET_CONFIG          0x01
#define SET_CONFIG          0x02
#define READ_VBATT          0x03
#define FIVE_BAUD_INIT      0x04
#define FAST_INIT           0x05
#define CLEAR_TX_BUFFER     0x07
#define CLEAR_RX_BUFFER     0x08
#define CLEAR_PERIODIC_MSGS 0x09
#define CLEAR_MSG_FILTERS   0x0A
#define READ_PROG_VOLTAGE   0x0E

// Config parameters
#define DATA_RATE      0x01
#define LOOPBACK       0x03
#define ISO15765_BS    0x1E
#define ISO15765_STMIN 0x1F

// SetProgrammingVoltage
#define SHORT_TO_GROUND 0xFFFFFFFE
#define VOLTAGE_OFF     0xFFFFFFFF

typedef struct
{
	uint32_t ProtocolID;
	uint32_t RxStatus;
	uint32_t TxFlags;
	uint32_t Timestamp;
	uint32_t DataSize;
	uint32_t ExtraDataIndex;
	unsigned char Data[4128];
} PASSTHRU_MSG;

typedef struct
{
	uint32_t Parameter;
	uint32_t Value;
} SCONFIG;

typedef struct
{
	uint32_t NumOfParams;
	SCONFIG *ConfigPtr;
} SCONFIG_LIST;

typedef struct
{
	uint32_t NumOfBytes;
	unsigned char *BytePtr;
} SBYTE_ARRAY;

#ifdef __cplusplus
extern "C" {
#endif

int32_t J2534_API PassThruOpen(void *pName, uint32_t *pDeviceID);
int32_t J2534_API PassThruClose(uint32_t DeviceID);
int32_t J2534_API PassThruConnect(uint32_t DeviceID, uint32_t ProtocolID,
                                  uint32_t Flags, uint32_t BaudRate, uint32_t *pChannelID);
int32_t J2534_API PassThruDisconnect(uint32_t ChannelID);
int32_t J2534_API PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
                                   uint32_t *pNumMsgs, uint32_t Timeout);
int32_t J2534_API PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
                                    uint32_t *pNumMsgs, uint32_t Timeout);
int32_t J2534_API PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
                                           uint32_t *pMsgID, uint32_t TimeInterval);
int32_t J2534_API PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t MsgID);
int32_t J2534_API PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType,
                                         PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg, uint32_t *pFilterID);
int32_t J2534_API PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID);
int32_t J2534_API PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber,
                                                uint32_t Voltage);
int32_t J2534_API PassThruReadVersion(uint32_t DeviceID, char *pFirmwareVersion,
                                      char *pDllVersion, char *pApiVersion);
int32_t J2534_API PassThruGetLastError(char *pErrorDescription);
int32_t J2534_API PassThruIoctl(uint32_t ChannelID, uint32_t IoctlID,
                                void *pInput, void *pOutput);

#ifdef __cplusplus
}
#endif

#endif // J2534_H
