#include "ddl.h"
#include "stdint.h"
#include "stddef.h"
#include "string.h"
#include "guic_gunma.h"
#include "flash.h"


GunMaSettingType gGunmaSetting = { 0 };


uint8_t gUart0RxDataBuf[64] = { 0 };
volatile uint8_t gUart0RxDataIdx = 0;
volatile GunmaState gGunmaState = GUNMA_STATE_IDLE;
uint8_t gGunmaLength = 0;

uint32_t  dataSectorBaseAddr = 0x7F00;

volatile GUANMA_IC_STATE gGunamICState = GUNMA_IC_STATE_IDLE;


void SoftUart_SendByte(uint8_t u8Data);
void DealWithNewGunma(uint8_t newGunma, uint8_t oldGunma);

/**
 * @brief CRC-8 校验 (多项式 0x07, CRC-8-CCITT)
 *        用于软件模拟串口的数据完整性校验
 */
static uint8_t gunma_crc8(const uint8_t* data, uint16_t len) {
	uint8_t crc = 0x00;
	uint8_t i;
	while (len--) {
		crc ^= *data++;
		for (i = 0; i < 8; i++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}



/**
  *  读取设置
  */
void readGunmaSetting() {

	memcpy(&gGunmaSetting, (uint8_t*)dataSectorBaseAddr, sizeof(GunMaSettingType));
	if (gGunmaSetting.gunmaDeviceId == 0xFFFF) {
		memset(&gGunmaSetting, 0, sizeof(gGunmaSetting));
	}

}


void writeGunmaSetting(uint8_t* value, uint32_t length) {

	///< FLASH目标扇区擦除
	while (Ok != Flash_SectorErase(dataSectorBaseAddr)) {
		;
	}

	for (int i = 0; i < length; i++) {
		Flash_WriteByte(dataSectorBaseAddr + i, *value);
		value++;
	}
}

void gunmaInit() {

	while (Ok != Flash_Init(1)) {
		;
	}
	readGunmaSetting();
}

/**
 *@brief 发送滚码协议数据
 *
 * @param cmd  滚码协议命令字
 * @param data gunma协议数据
 * @param len  gunma协议数据长度，不包含命令字，仅仅是数据长度
 */
void gunmaProtocol_Send(GunmaProtocolCmd cmd, uint8_t* data, uint16_t len) {
	if (data == NULL || len > 60 || len == 0) {
		return; // 数据为空或长度超过协议限制，直接返回
	}
	// CRC-8 校验: 覆盖 [cmd, data[0..len-1]]
	uint8_t tmp[64];
	tmp[0] = (uint8_t)cmd;
	memcpy(tmp + 1, data, len);
	uint8_t crc = gunma_crc8(tmp, len + 1);

	SoftUart_SendByte(0x55); // 起始字节
	SoftUart_SendByte(0xAA); // ACK
	SoftUart_SendByte(len + 2);  // 长度: cmd(1) + data(len) + crc(1)
	SoftUart_SendByte(cmd); // 命令字节
	for (int i = 0; i < len; i++) {
		SoftUart_SendByte(data[i]);
	}
	SoftUart_SendByte(crc); // CRC-8
}


/**
 * @brief 处理滚码协议
 */
void gunmaProtocol_Process() {
	DEBUG("gunmaProtocol_Process");
	if (gGunmaState != GUNMA_STATE_WAIT_END) {
		return; // 只有在等待数据处理完成时才进行处理
	}
	// CRC-8 校验: 最后1字节为CRC，前面为数据
	if (gGunmaLength < 2) {
		return; // 至少需要cmd(1) + crc(1)
	}
	uint8_t received_crc = gUart0RxDataBuf[gGunmaLength - 1];
	uint8_t calculated_crc = gunma_crc8(gUart0RxDataBuf, gGunmaLength - 1);
	if (received_crc != calculated_crc) {
		DEBUG("CRC mismatch! calc:%02x recv:%02x\n", calculated_crc, received_crc);
		return; // CRC校验失败，丢弃
	}

	uint8_t cmd = gUart0RxDataBuf[0]; // 命令字在数据的第一个字节
	uint8_t buf[64] = { 0 };
	uint8_t newGunma = 0;
	uint8_t oldGunma = 0;
	DEBUG("gunmaProtocol_Process %d\n", cmd);
	switch (cmd) {

	case GUNMA_CMD_SETTING:
		// gGunmaLength 含 cmd(1) + data + crc(1), 实际数据长度 = gGunmaLength - 2
		if ((gGunmaLength - 2) != sizeof(GunMaSettingType)) {
			buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_SYSTEM_ERROR;
			gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 1);
			return;
		}
		memcpy(&gGunmaSetting, gUart0RxDataBuf + 1, gGunmaLength - 2);
		writeGunmaSetting(gUart0RxDataBuf + 1, gGunmaLength - 2);
		delay1ms(10);
		DEBUG("%d, %d, %d", gGunmaSetting.device_type, gGunmaSetting.gunmaDeviceId, gGunmaSetting.sector_no);
		break;

	case GUNMA_CMD_RECEIVE_CARDNO:
		/* code */

		break;
	case GUNMA_CMD_IC_VERIFY_RESULT:
		/* ic卡侧滚码 */

		break;
	case GUNMA_CMD_GUNMA_VERIFY_RESULT:
		if (gUart0RxDataBuf[1] == GUNMA_VERIFY_SUCCESS) {
			newGunma = gUart0RxDataBuf[2];

			// 旧滚码，当滚码设备id变更时，设备侧根据卡号查出的正确滚码，
			// 返回需要ic卡侧验证正确性，如果能找到滚码，说明卡是对的，否则卡是复制卡

			oldGunma = gUart0RxDataBuf[4];
			__disable_irq();
			DealWithNewGunma(newGunma, oldGunma);
			__enable_irq();
			// 写卡完成后保持 gGunamICState 非 IDLE
			// rc522 读卡循环检测到 != IDLE 后会等待 3000ms 冷却期
			// 冷却期内不会重新寻卡→不会触发二次写入→不会报 WRITE_FAIL
		}
	break;		case GUNMA_CMD_REBOOT:
		__disable_irq();
		SCB->AIRCR = 0x05FA0004;  // 系统复位
		while (1);
	break;    default:
		break;
	}
}


void gunmaDataProcess(uint8_t rxData) {
	//printf(" %02x %d, %d \n", rxData, gGunmaState, gUart0RxDataIdx);
	if (rxData == 0x55) {
		gGunmaState = GUNMA_STATE_WAIT_ACK0;
		memset(gUart0RxDataBuf, 0, sizeof(gUart0RxDataBuf));
		gUart0RxDataIdx = 0;
		return;
	}

	switch (gGunmaState) {

	case GUNMA_STATE_IDLE:
		/*
	if (rxData == 0x55) {
		gGunmaState = GUNMA_STATE_WAIT_ACK0;
		memset(gUart0RxDataBuf, 0, sizeof(gUart0RxDataBuf));
		gUart0RxDataIdx = 0;
	} */
		break;
	case GUNMA_STATE_WAIT_ACK0:
		if (rxData == 0xAA) {
			gGunmaState = GUNMA_STATE_WAIT_ACK1;
		} else {
			gGunmaState = GUNMA_STATE_IDLE;
		}
		break;
	case GUNMA_STATE_WAIT_ACK1:
		gGunmaLength = rxData;
		gGunmaState = GUNMA_STATE_WAIT_DATA;
		break;
	case GUNMA_STATE_WAIT_DATA:
		gUart0RxDataBuf[gUart0RxDataIdx++] = rxData;
		if (gUart0RxDataIdx >= gGunmaLength) {
			// 数据接收完成，进行处理
			gGunmaState = GUNMA_STATE_WAIT_END;
			gunmaProtocol_Process();
			gGunmaState = GUNMA_STATE_IDLE;
		}
		break;
	case GUNMA_STATE_WAIT_END: // 什么也不干，等待数据处理完成后重置状态
		break;
	default:
		gGunmaState = GUNMA_STATE_IDLE;
		break;
	}
}


