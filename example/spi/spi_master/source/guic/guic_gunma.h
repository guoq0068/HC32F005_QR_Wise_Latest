#ifndef __IGUIC_GUNMA_H__
#define __IGUIC_GUNMA_H__
#include "stdint.h"

#ifdef __IGUIC_DEBUG__
#define DEBUG(...) printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

typedef enum
{
    GUNMA_STATE_IDLE = 0,
    GUNMA_STATE_WAIT_ACK0,  // 接收到0xE5
    GUNMA_STATE_WAIT_ACK1,  // 接收到0xAA
    GUNMA_STATE_WAIT_DATA,  // 处理数据
    GUNMA_STATE_WAIT_END,
} GunmaState;

typedef enum {
    GUNMA_IC_STATE_IDLE = 0,
    GUNMA_IC_STATE_SELECT_CARD,      // 选卡
    GUNMA_IC_STATE_SENDING_IC_CARD,  // 发送卡号
    GUNMA_IC_STATE_SENDING_IC_VERIFY // 验证
} GUANMA_IC_STATE;

typedef enum {
    GUNMA_CMD_NONE = 0,       // 无命令
    GUNMA_CMD_SETTING,        // 设置命令
    GUNMA_CMD_RECEIVE_CARDNO,      // 接收卡号
    GUNMA_CMD_GUNMA_VERIFY_RESULT, // 滚码验证结果
    GUNMA_CMD_IC_VERIFY_RESULT,  // ic卡验证结果,x
    GUNMA_CMD_REBOOT,            // 重启命令
} GunmaProtocolCmd;

typedef enum {
    GUNMA_VERIFY_SUCCESS = 0,       // 滚码验证成功无命令
    GUNMA_VERIFY_FAIL_FUZHIKA,      // 复制卡
    GUNMA_VERIFY_DO_NOTHING,          // 滚码验证成功有命令，设备不处理
} GunmaVerifyResult;

typedef enum {
    IC_VERIFY_SUCCESS = 0,       // 滚码验证成功无命令
    IC_VERIFY_FAIL_AUTH_ERROR,   // 未授权ic卡
    IC_VERIFY_FAIL_FUZHIKA,      // 复制卡
} ICVerifyResult;

typedef enum {
    INVALID_CARD_WRITE_0_SECTOR = 1,  // 能写0扇区得卡，uid，cuid等
    INVALID_CARD_BACK_DOOR, 					// 后门卡
    INVALID_CARD_NFC_CPU,   						// cpu卡
    INVALID_CARD_7_UID								// 7位UID
} InValidCardType;

typedef enum {
    GUNMA_IC_VERIFY_RESULT_SUCCESS = 0,       // 验证成功
    GUNMA_IC_VERIFY_RESULT_FAIL_GUNMA_WRITE_FAIL,
    GUNMA_IC_VERIFY_RESULT_FAIL_FUZHI_CARD,
    GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD,  // 非法卡
    GUNMA_IC_VERIFY_RESULT_FAIL_SYSTEM_ERROR	 // 系统错误
} GunmaICVerifyResult;



typedef struct __gunmaSettingType {
    uint8_t key_buf[12];
    uint8_t device_type;
    uint8_t sector_no;
    uint16_t gunmaDeviceId;
} GunMaSettingType;


#define SAK_CPU_NFC  0x20

extern volatile GUANMA_IC_STATE gGunamICState;

void gunmaProtocol_Process();
void gunmaDataProcess(uint8_t rxData);
void gunmaProtocol_Send(GunmaProtocolCmd cmd, uint8_t* data, uint16_t len);
void gunmaInit();

#endif