#include "rc522_function.h"
#include "spi.h"
#include "guic_gunma.h"
#include "wdt.h"



unsigned char  data1[16] = { 0x01,0x00,0x00,0x00,0xFE,0xFF,0xFF,0xFF,0x01,0x00,0x00,0x00,0x01,0xFE,0x01,0xFE };//写金额
//M1卡的某一块写为如下格式，则该块为钱包，可接收扣款和充值命令
//4字节金额（低字节在前）＋4字节金额取反＋4字节金额＋1字节块地址＋1字节块地址取反＋1字节块地址＋1字节块地址取反 
unsigned char  data2[4] = { 0x01,0,0,0 };//充值金额
unsigned char  DefaultKey[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
unsigned char g_ucTempbuf[32];    //备份的钱包
unsigned char status, i;
unsigned int temp;


static enum { STATE_SCAN, STATE_WAIT_FEEDBACK, STATE_COOLDOWN } current_state = STATE_SCAN;
static uint8_t active_uid[5] = { 0 };
static volatile uint32_t wait_start_tick = 0;

float buffer = 0;

int exti_t = 1;

uint32_t Get_SysTick(void);
char PcdComMF522(u8 ucCommand, u8* pInData, u8 ucInLenByte, u8* pOutData, u32* pOutLenBit);
void CalulateCRC(u8* pIndata, u8 ucLen, u8* pOutData);

extern GunMaSettingType gGunmaSetting;

typedef struct __IC_NODE__ {
	uint16_t  gunma_device_id;
	uint8_t   gunma;      // 最新滚码
	uint8_t   gunma_old;  // 上一次滚码(双滚码容错方案)
} IC_NODE_TYPE;

typedef struct __IC_BLOCK__ {
	IC_NODE_TYPE  nodes[4];
} IC_BLOCK_TYPE;

typedef struct __IC_SECTOR__ {
	IC_BLOCK_TYPE  block[4];
} IC_SECTOR_TYPE;

IC_SECTOR_TYPE gIcSectorBuf = { 0 };
/*
 * 函数名：SPI_RC522_SendByte
 * 描述  ：向RC522发送1 Byte 数据
 * 输入  ：byte，要发送的数据
 * 返回  : RC522返回的数据
 * 调用  ：内部调用
 */
void SPI_RC522_SendByte(u8 byte) {
	Spi_SetCS(M0P_SPI, FALSE);
	//while(Spi_GetStatus(M0P_SPI, SpiIf) == FALSE);
	Spi_SendData(M0P_SPI, byte);
	//delay(5);
	while (Spi_GetStatus(M0P_SPI, SpiIf) == FALSE) {}; //等待发送完成
	///< 结束通信
	Spi_SetCS(M0P_SPI, TRUE);

}


/*
 * 函数名：SPI_RC522_ReadByte
 * 描述  ：从RC522发送1 Byte 数据
 * 输入  ：无
 * 返回  : RC522返回的数据
 * 调用  ：内部调用
 */
u8 SPI_RC522_ReadByte(void) {
	u8 counter;
	u8 SPI_Data;
	Spi_SetCS(M0P_SPI, FALSE);
	//while(Spi_GetStatus(M0P_SPI, SpiIf) == FALSE);//等待接收缓冲器非空
	SPI_Data = Spi_ReceiveData(M0P_SPI);  //读取接收到的数据
	Spi_SetCS(M0P_SPI, TRUE);

	return SPI_Data;

}


/*
 * 函数名：ReadRawRC
 * 描述  ：读RC522寄存器
 * 输入  ：ucAddress，寄存器地址
 * 返回  : 寄存器的当前值
 * 调用  ：内部调用
 */
u8 ReadRawRC(u8 ucAddress) {
	u8 ucAddr, ucReturn;


	ucAddr = ((ucAddress << 1) & 0x7E) | 0x80;

	//macRC522_CS_Enable();
	Spi_SetCS(M0P_SPI, FALSE);

	Spi_SendData(M0P_SPI, ucAddr);
	// 等待传输完成（发送+接收）
	while (Spi_GetStatus(M0P_SPI, SpiIf) == FALSE);
	// 读取接收缓冲区（虽然是无效数据，但必须读，避免溢出）
	ucReturn = Spi_ReceiveData(M0P_SPI);

	// --------------------- 发送空字节并接收寄存器值（第二个字节） ---------------------
	// 先等待发送缓冲区为空，再发送（关键修复2）
	//while (Spi_GetStatus(M0P_SPI, SpiTxEmpty) == FALSE);
	Spi_SendData(M0P_SPI, 0); // 发送空字节，触发RC522返回寄存器值
	// 等待传输完成
	while (Spi_GetStatus(M0P_SPI, SpiIf) == FALSE);
	// 读取真正的寄存器值
	ucReturn = Spi_ReceiveData(M0P_SPI);


	Spi_SetCS(M0P_SPI, TRUE);


	return ucReturn;


}


/*
 * 函数名：WriteRawRC
 * 描述  ：写RC522寄存器
 * 输入  ：ucAddress，寄存器地址
 *         ucValue，写入寄存器的值
 * 返回  : 无
 * 调用  ：内部调用
 */
void WriteRawRC(u8 ucAddress, u8 ucValue) {
	u8 ucAddr;


	ucAddr = (ucAddress << 1) & 0x7E;
	//macRC522_CS_Enable();
	Spi_SetCS(M0P_SPI, FALSE);
	// delay10us(1);
	Spi_SendData(M0P_SPI, ucAddr);
	while (Spi_GetStatus(M0P_SPI, SpiIf) == FALSE) {}; //等待发送完成
	Spi_ReceiveData(M0P_SPI);
	///< 结束通信


	Spi_SendData(M0P_SPI, ucValue);
	while (Spi_GetStatus(M0P_SPI, SpiIf) == FALSE) {}; //等待发送完成
	Spi_ReceiveData(M0P_SPI);
	///< 结束通信	
	Spi_SetCS(M0P_SPI, TRUE);
	//macRC522_CS_Disable();	


}


/*
 * 函数名：SetBitMask
 * 描述  ：对RC522寄存器置位
 * 输入  ：ucReg，寄存器地址
 *         ucMask，置位值
 * 返回  : 无
 * 调用  ：内部调用
 */
void SetBitMask(u8 ucReg, u8 ucMask) {
	u8 ucTemp;


	ucTemp = ReadRawRC(ucReg);

	WriteRawRC(ucReg, ucTemp | ucMask);         // set bit mask


}


/*
 * 函数名：ClearBitMask
 * 描述  ：对RC522寄存器清位
 * 输入  ：ucReg，寄存器地址
 *         ucMask，清位值
 * 返回  : 无
 * 调用  ：内部调用
 */
void ClearBitMask(u8 ucReg, u8 ucMask) {
	u8 ucTemp;


	ucTemp = ReadRawRC(ucReg);

	WriteRawRC(ucReg, ucTemp & (~ucMask));  // clear bit mask


}



/*
 * 函数名：PcdAntennaOn
 * 描述  ：开启天线
 * 输入  ：无
 * 返回  : 无
 * 调用  ：内部调用
 */
void PcdAntennaOn(void) {
	u8 uc;

	uc = ReadRawRC(TxControlReg);
	DEBUG("uc is %d", uc);
	if (!(uc & 0x03))
		SetBitMask(TxControlReg, 0x03);


}


/*
 * 函数名：PcdAntennaOff
 * 描述  ：关闭天线
 * 输入  ：无
 * 返回  : 无
 * 调用  ：内部调用
 */
void PcdAntennaOff(void) {
	ClearBitMask(TxControlReg, 0x03);


}


/**
 * @brief  微秒级延时函数
 * @param  u32Us 延时的微秒数
 * @note   基于 24MHz 主频计算，主频不同需调整循环次数
 */
void Delay_us(uint32_t u32Us) {
	// 24MHz 下，1us 大约执行 24 个时钟周期
	// while 循环包含判断和自减，大约占用 3~4 个周期
	// 因此这里设置为 6~8 左右比较接近 1us
	uint32_t u32Count = u32Us * 8;

	while (u32Count--) {
		__nop(); // 保证编译器不优化掉空循环
	}

}

/*
 * 函数名：PcdReset
 * 描述  ：复位RC522
 * 输入  ：无
 * 返回  : 无
 * 调用  ：外部调用
 */
void PcdReset(void) {
	macRC522_Reset_Disable();

	Delay_us(1);
	delay1ms(10);

	macRC522_Reset_Enable();

	Delay_us(1);
	delay1ms(50);

	macRC522_Reset_Disable();
	delay1ms(50);
	Delay_us(1);

	WriteRawRC(CommandReg, 0x0f);

	u8 temp = ReadRawRC(CommandReg);
	DEBUG("temp value 1 %02x", temp);
	while (temp & 0x10) {
		temp = ReadRawRC(CommandReg);
		DEBUG("temp value 2 %02x", temp);

	};

	Delay_us(1);
	delay1ms(50);



	WriteRawRC(ModeReg, 0x3D);            //定义发送和接收常用模式 和Mifare卡通讯，CRC初始值0x6363

	WriteRawRC(TReloadRegL, 30);          //16位定时器低位  


	WriteRawRC(TReloadRegH, 0);			     //16位定时器高位

	WriteRawRC(TModeReg, 0x8D);				   //定义内部定时器的设置

	WriteRawRC(TPrescalerReg, 0x3E);			 //设置定时器分频系数

	WriteRawRC(TxAutoReg, 0x40);				   //调制发送信号为100%ASK	

	/*
	WriteRawRC(TxControlReg, 0x83);

	temp = ReadRawRC(VersionReg);
	DEBUG("versionReg value is %02x", temp); */
}

u8 gc_ver = 0;
/*
 * 函数名：M500PcdConfigISOType
 * 描述  ：设置RC522的工作方式
 * 输入  ：ucType，工作方式
 * 返回  : 无
 * 调用  ：外部调用
 */
void M500PcdConfigISOType(u8 ucType) {
	uint8_t ver;

	if (ucType == 'A')                     //ISO14443_A
	{
		ClearBitMask(Status2Reg, 0x08);

		WriteRawRC(ModeReg, 0x3D);//3F

		WriteRawRC(RxSelReg, 0x86);//84

		WriteRawRC(RFCfgReg, 0x7F);   //4F

		WriteRawRC(TReloadRegL, 30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 

		WriteRawRC(TReloadRegH, 0);

		WriteRawRC(TModeReg, 0x8D);

		WriteRawRC(TPrescalerReg, 0x3E);

		Delay_us(2);
		delay1ms(10);
		PcdAntennaOff();

		delay1ms(10);
		PcdAntennaOn();//开天线

		delay1ms(10);


	}


}


/*
 * 函数名：PcdComMF522
 * 描述  ：通过RC522和ISO14443卡通讯
 * 输入  ：ucCommand，RC522命令字
 *         pInData，通过RC522发送到卡片的数据
 *         ucInLenByte，发送数据的字节长度
 *         pOutData，接收到的卡片返回数据
 *         pOutLenBit，返回数据的位长度
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：内部调用
 */
char PcdComMF522(u8 ucCommand, u8* pInData, u8 ucInLenByte, u8* pOutData, u32* pOutLenBit) {
	char cStatus = MI_ERR;
	u8 ucIrqEn = 0x00;
	u8 ucWaitFor = 0x00;
	u8 ucLastBits;
	u8 ucN;
	u32 ul;


	switch (ucCommand) {
	case PCD_AUTHENT:		//Mifare认证
		ucIrqEn = 0x12;		//允许错误中断请求ErrIEn  允许空闲中断IdleIEn
		ucWaitFor = 0x10;		//认证寻卡等待时候 查询空闲中断标志位
		break;

	case PCD_TRANSCEIVE:		//接收发送 发送接收
		ucIrqEn = 0x77;		//允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
		ucWaitFor = 0x30;		//寻卡等待时候 查询接收中断标志位与 空闲中断标志位
		break;

	default:
		break;

	}

	WriteRawRC(ComIEnReg, ucIrqEn | 0x80);		//IRqInv置位管脚IRQ与Status1Reg的IRq位的值相反 
	ClearBitMask(ComIrqReg, 0x80);			//Set1该位清零时，CommIRqReg的屏蔽位清零
	WriteRawRC(CommandReg, PCD_IDLE);		//写空闲命令
	SetBitMask(FIFOLevelReg, 0x80);			//置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除

	for (ul = 0; ul < ucInLenByte; ul++)
		WriteRawRC(FIFODataReg, pInData[ul]);    		//写数据进FIFOdata

	WriteRawRC(CommandReg, ucCommand);					//写命令

	Wdt_Feed();
	if (ucCommand == PCD_TRANSCEIVE)
		SetBitMask(BitFramingReg, 0x80);  				//StartSend置位启动数据发送 该位与收发命令使用时才有效

	ul = 10000;//根据时钟频率调整，操作M1卡最大等待时间25ms

	do 														//认证 与寻卡等待时间	
	{
		ucN = ReadRawRC(ComIrqReg);							//查询事件中断
		ul--;
	} while ((ul != 0) && (!(ucN & 0x01)) && (!(ucN & ucWaitFor)));		//退出条件i=0,定时器中断，与写空闲命令
	Wdt_Feed();
	ClearBitMask(BitFramingReg, 0x80);					//清理允许StartSend位


	if (ul != 0) {
		u8 temp = ReadRawRC(ErrorReg);


		if (!(temp & 0x1B))			//读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
		{

			cStatus = MI_OK;

			if (ucN & ucIrqEn & 0x01)					//是否发生定时器中断
				cStatus = MI_NOTAGERR;

			if (ucCommand == PCD_TRANSCEIVE) {
				ucN = ReadRawRC(FIFOLevelReg);			//读FIFO中保存的字节数

				ucLastBits = ReadRawRC(ControlReg) & 0x07;	//最后接收到得字节的有效位数

				if (ucLastBits)
					*pOutLenBit = (ucN - 1) * 8 + ucLastBits;   	//N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
				else
					*pOutLenBit = ucN * 8;   					//最后接收到的字节整个字节有效

				if (ucN == 0)
					ucN = 1;

				if (ucN > MAXRLEN)
					ucN = MAXRLEN;

				for (ul = 0; ul < ucN; ul++)
					pOutData[ul] = ReadRawRC(FIFODataReg);

			}

		}

		else {
			//DEBUG("error reg %02x\n", temp);
			cStatus = MI_ERR;
		}
	}

	SetBitMask(ControlReg, 0x80);           // stop timer now
	WriteRawRC(CommandReg, PCD_IDLE);


	return cStatus;


}

/**
 * @brief 识别是否为复制卡
 * @return 0: 原厂正品卡, 1: 复制卡/后门卡
 */
uint8_t Check_Is_CloneCard(void) {

	uint8_t status;
	uint8_t ucComMF522Buf[MAXRLEN];
	uint32_t ulLen; // 必须是 u32


	// 1. 强制重置状态，关闭加密和校验
	WriteRawRC(CommandReg, PCD_IDLE);
	WriteRawRC(ComIrqReg, 0x7F);
	WriteRawRC(FIFOLevelReg, 0x80); // 清空FIFO

	ClearBitMask(Status2Reg, 0x08);  // 彻底关掉加密
	WriteRawRC(TxModeReg, 0x00);     // 禁用发送CRC
	WriteRawRC(RxModeReg, 0x00);     // 禁用接收CRC

	// 2. 发送魔术指令 0x40 (必须是 7 bits)
	WriteRawRC(BitFramingReg, 0x07);
	ucComMF522Buf[0] = 0x40;


	// 注意：探测非复制卡时，这里一定会 MI_ERR (超时)，这是正常的  PCD_TRANSCEIVE
	status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &ulLen);
	// DEBUG("status %d\n", status);
// 探测完第一步，无论成功与否，都要检查结果
// 复制卡通常回传 0x0A (4 bits)，所以 ulLen 可能是 4
	if (status == MI_OK && (ucComMF522Buf[0] & 0x0F) == 0x0A) {
		goto CLONE_DETECTED;
	}

	// 3. 继续尝试 0x43 (8 bits)
	WriteRawRC(BitFramingReg, 0x00);
	ucComMF522Buf[0] = 0x43;
	status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &ulLen);
	//DEBUG("43 status %d\n", status);
	if (status == MI_OK && ucComMF522Buf[0] == 0x0A) {
		goto CLONE_DETECTED;
	}

	// 正常退出（原厂卡）

	WriteRawRC(CommandReg, PCD_IDLE);
	WriteRawRC(BitFramingReg, 0x00); // 强制回 8 位 
	//WriteRawRC(TxModeReg, 0x80);     // 重新开启 CRC  
	//WriteRawRC(RxModeReg, 0x80); 
	WriteRawRC(FIFOLevelReg, 0x80);  // 清空 FIFO 
	ClearBitMask(Status2Reg, 0x08);  // 关掉加密位	


	return 0;

CLONE_DETECTED:
	WriteRawRC(CommandReg, PCD_IDLE);
	WriteRawRC(BitFramingReg, 0x00); // 强制回 8 位
	//WriteRawRC(TxModeReg, 0x80);     // 重新开启 CRC
	//WriteRawRC(RxModeReg, 0x80);
	WriteRawRC(FIFOLevelReg, 0x80);  // 清空 FIFO
	ClearBitMask(Status2Reg, 0x08);  // 关掉加密位
	return 1;
}

/**
 * @brief 安全修改后门卡的 UID
 * @param pNewUID: 4字节新UID
 * @return 1: 成功, 0: 失败
 */
uint8_t Safe_Write_UID_Backdoor() {
	u8 status;
	u8 ucBuf[MAXRLEN];
	u32 ulLen;
	u8 old_block0[18] = { 0 };
	u8 i;

	// --- 第一步：获取原始 Block 0 数据 ---
	// 注意：后门卡在发送魔术序列后可以直接读取 Block 0 而无需认证

	status = PcdRead(0, old_block0);
	//DEBUG("safe status  3 %d  %02X %02X %02X %02X\n", status, old_block0[0], old_block0[1], old_block0[2], old_block0[3]);
	if (status != MI_OK) return 0;

	// --- 第二步：构造新的 Block 0 ---
	// 保留原始 block0 的后 11 字节（厂商信息、SAK 等）
	// 只修改前 4 字节 UID 和第 5 字节 BCC

	PcdAnticoll(old_block0); // 重新防冲突（可选，增加稳定性）
	PcdSelect(old_block0);   // 必须重新选中卡片！
	old_block0[0] = old_block0[0] + 1;
	//old_block0[1] = pNewUID[1];
	//old_block0[2] = pNewUID[2];
	//old_block0[3] = pNewUID[3];
	old_block0[4] = (old_block0[0] ^ old_block0[1] ^ old_block0[2] ^ old_block0[3]) & 0xff; // 核心：计算BCC
	CalulateCRC(old_block0, 16, &old_block0[16]);

	// --- 第三步：再次触发后门写模式 ---
	// 很多卡片在 Read 后需要重新发送魔术序列才能开启 Write
	WriteRawRC(CommandReg, PCD_IDLE);
	WriteRawRC(BitFramingReg, 0x00);
	WriteRawRC(TxModeReg, 0x00); // 确保关闭硬件自动CRC
	WriteRawRC(RxModeReg, 0x00);
	WriteRawRC(FIFOLevelReg, 0x80);

	WriteRawRC(BitFramingReg, 0x07);
	ucBuf[0] = 0x40;
	status = PcdComMF522(PCD_TRANSCEIVE, ucBuf, 1, ucBuf, &ulLen);

	//DEBUG("safe status  5 %d ulLen %d\n", status, ulLen);

	WriteRawRC(BitFramingReg, 0x00);
	ucBuf[0] = 0x43;
	status = PcdComMF522(PCD_TRANSCEIVE, ucBuf, 1, ucBuf, &ulLen);
	//DEBUG("safe status  6 %d\n", status);

// --- 第四步：发送写指令 0xA0 ---
	ucBuf[0] = 0xA0;
	ucBuf[1] = 0x00; // Block 0
	// 如果你的 PcdComMF522 内部不带软件 CRC 计算，
	CalulateCRC(ucBuf, 2, &ucBuf[2]);
	// 这里需要手动给 0xA0 0x00 计算并追加 2 字节 CRC
	status = PcdComMF522(PCD_TRANSCEIVE, ucBuf, 4, ucBuf, &ulLen);
	//DEBUG("safe status  4 %d\n", status);
	if (status != MI_OK || (ucBuf[0] & 0x0F) != 0x0A) return 0;

	// --- 第五步：写入 16 字节数据 ---
	status = PcdComMF522(PCD_TRANSCEIVE, old_block0, 18, ucBuf, &ulLen);
	//DEBUG("safe status  7 %d\n", status);
	// 环境大扫除
	WriteRawRC(BitFramingReg, 0x00);

	if (status == MI_OK) {
		//DEBUG("New UID Write Success!\n");
		return 1;
	}
	return 0;
}

/**
 * @brief 通过尝试修改 Block 0 来检测 CUID 类型复制卡
 * @param pUID: 选卡得到的 4 字节 UID
 * @return 1: 复制卡(Block0可改写), 0: 原厂卡(Block0只读)
 */
uint8_t Check_By_Writing_Block0(u8* pUID) {
	u8 status;
	u8 block0_data[16];
	u8 test_data[16];
	// 大部分 CUID 卡出厂默认密钥是全 0xFF
	u8 defaultKey[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	// 1. 认证 0 块 (Sector 0)
	// 必须先经过 PcdSelect 选中卡片后才能认证
	status = PcdAuthState(PICC_AUTHENT1A, 0, defaultKey, pUID);
	if (status != MI_OK) {
		//DEBUG("#check1 %d\n", status);
		// 关键修复：认证失败时，M1卡进入死锁状态+RC522的Crypto1On可能被置位
		// 必须清理RC522状态并做RF场复位，否则后续读卡全部失败
		WriteRawRC(CommandReg, PCD_IDLE);
		ClearBitMask(Status2Reg, 0x08);  // 清除Crypto1On
		WriteRawRC(FIFOLevelReg, 0x80);  // 清空FIFO
		return 0; // 认证失败（可能是加密卡），无法通过写块判断
	}

	// 2. 读取原始 Block 0 数据
	status = PcdRead(0, block0_data);
	if (status != MI_OK) {
		//DEBUG("#check 2 %d\n", status);
		return 0;
	}
	// 3. 构造测试数据：只修改厂商信息外的一个无关字节，或者取反第一个 UID 字节
	memcpy(test_data, block0_data, 16);
	test_data[0] ^= 0xFF; // 临时翻转 UID 第一个字节

	// 4. 尝试写入 Block 0
	status = PcdWrite(0, test_data);

	if (status == MI_OK) {
		// --- 关键：写入成功说明是复制卡，必须立刻把原始数据还原 ---
		PcdWrite(0, block0_data);
		return 1;
	} else {
		// 写入失败说明是原厂只读 Block 0
				//DEBUG("#3\n");
		return 0;
	}
}


/*
 * 函数名：PcdRequest
 * 描述  ：寻卡
 * 输入  ：ucReq_code，寻卡方式
 *                     = 0x52，寻感应区内所有符合14443A标准的卡
 *                     = 0x26，寻未进入休眠状态的卡
 *         pTagType，卡片类型代码
 *                   = 0x4400，Mifare_UltraLight
 *                   = 0x0400，Mifare_One(S50)
 *                   = 0x0200，Mifare_One(S70)
 *                   = 0x0800，Mifare_Pro(X))
 *                   = 0x4403，Mifare_DESFire
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdRequest(u8 ucReq_code, u8* pTagType) {
	char cStatus;
	u8 ucComMF522Buf[MAXRLEN];
	u32 ulLen;


	ClearBitMask(Status2Reg, 0x08);	//清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
	WriteRawRC(BitFramingReg, 0x07);	//	发送的最后一个字节的 七位
	SetBitMask(TxControlReg, 0x03);	//TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号

	ucComMF522Buf[0] = ucReq_code;		//存入 卡片命令字

	cStatus = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &ulLen);	//寻卡  

	if ((cStatus == MI_OK) && (ulLen == 0x10))	//寻卡成功返回卡类型 
	{
		*pTagType = ucComMF522Buf[0];
		*(pTagType + 1) = ucComMF522Buf[1];
	}

	else
		cStatus = MI_ERR;


	return cStatus;


}


/*
 * 函数名：PcdAnticoll
 * 描述  ：防冲撞
 * 输入  ：pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdAnticoll(u8* pSnr) {
	char cStatus;
	u8 uc, ucSnr_check = 0;
	u8 ucComMF522Buf[MAXRLEN];
	u32 ulLen;


	ClearBitMask(Status2Reg, 0x08);		//清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
	WriteRawRC(BitFramingReg, 0x00);		//清理寄存器 停止收发
	ClearBitMask(CollReg, 0x80);			//清ValuesAfterColl所有接收的位在冲突后被清除

	ucComMF522Buf[0] = 0x93;	//卡片防冲突命令
	ucComMF522Buf[1] = 0x20;

	cStatus = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &ulLen);//与卡片通信

	if (cStatus == MI_OK)		//通信成功
	{
		for (uc = 0; uc < 4; uc++) {
			*(pSnr + uc) = ucComMF522Buf[uc];			//读出UID
			ucSnr_check ^= ucComMF522Buf[uc];
		}

		if (ucSnr_check != ucComMF522Buf[uc])
			cStatus = MI_ERR;

	}

	SetBitMask(CollReg, 0x80);


	return cStatus;


}


/*
 * 函数名：CalulateCRC
 * 描述  ：用RC522计算CRC16
 * 输入  ：pIndata，计算CRC16的数组
 *         ucLen，计算CRC16的数组字节长度
 *         pOutData，存放计算结果存放的首地址
 * 返回  : 无
 * 调用  ：内部调用
 */
void CalulateCRC(u8* pIndata, u8 ucLen, u8* pOutData) {
	u8 uc, ucN;


	ClearBitMask(DivIrqReg, 0x04);

	WriteRawRC(CommandReg, PCD_IDLE);

	SetBitMask(FIFOLevelReg, 0x80);

	for (uc = 0; uc < ucLen; uc++)
		WriteRawRC(FIFODataReg, *(pIndata + uc));

	WriteRawRC(CommandReg, PCD_CALCCRC);

	uc = 0xFF;

	do {
		ucN = ReadRawRC(DivIrqReg);
		uc--;
	} while ((uc != 0) && !(ucN & 0x04));

	pOutData[0] = ReadRawRC(CRCResultRegL);
	pOutData[1] = ReadRawRC(CRCResultRegM);


}


/*
 * 函数名：PcdSelect
 * 描述  ：选定卡片
 * 输入  ：pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdSelect(u8* pSnr) {
	char ucN;
	u8 uc;
	u8 ucComMF522Buf[MAXRLEN];
	u32  ulLen = 0;


	ucComMF522Buf[0] = PICC_ANTICOLL1;
	ucComMF522Buf[1] = 0x70;
	ucComMF522Buf[6] = 0;

	for (uc = 0; uc < 4; uc++) {
		ucComMF522Buf[uc + 2] = *(pSnr + uc);
		ucComMF522Buf[6] ^= *(pSnr + uc);
	}

	CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

	ClearBitMask(Status2Reg, 0x08);



	ucN = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &ulLen);

	// DEBUG("ulLen%d\n", ulLen);
	if ((ucN == MI_OK) && (ulLen == 0x18))
		ucN = ucComMF522Buf[0];
	else
		ucN = MI_ERR;


	return ucN;


}


/*
 * 函数名：PcdAuthState
 * 描述  ：验证卡片密码
 * 输入  ：ucAuth_mode，密码验证模式
 *                     = 0x60，验证A密钥
 *                     = 0x61，验证B密钥
 *         u8 ucAddr，块地址
 *         pKey，密码
 *         pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdAuthState(u8 ucAuth_mode, u8 ucAddr, u8* pKey, u8* pSnr) {
	char cStatus;
	u8 uc, ucComMF522Buf[MAXRLEN];
	u32 ulLen;


	ucComMF522Buf[0] = ucAuth_mode;
	ucComMF522Buf[1] = ucAddr;

	for (uc = 0; uc < 6; uc++)
		ucComMF522Buf[uc + 2] = *(pKey + uc);

	for (uc = 0; uc < 4; uc++)
		ucComMF522Buf[uc + 8] = *(pSnr + uc);

	cStatus = PcdComMF522(PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &ulLen);



	u8 temp = ReadRawRC(Status2Reg);

	DEBUG("Auth state %02x status2reg %02x\n", cStatus, temp);

	if ((cStatus != MI_OK) || (!(temp & 0x08)))
		cStatus = MI_ERR;


	return cStatus;


}


/*
 * 函数名：PcdWrite
 * 描述  ：写数据到M1卡一块
 * 输入  ：u8 ucAddr，块地址
 *         pData，写入的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdWrite(u8 ucAddr, u8* pData) {
	char cStatus;
	u8 uc, ucComMF522Buf[MAXRLEN];
	u32 ulLen;


	ucComMF522Buf[0] = PICC_WRITE;
	ucComMF522Buf[1] = ucAddr;

	CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

	cStatus = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &ulLen);

	DEBUG("cStatus0 %d  ulLen %d  ucCom %d\n", cStatus, ulLen, ucComMF522Buf[0] & 0x0F);

	if ((cStatus != MI_OK) || (ulLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
		cStatus = MI_ERR;

	DEBUG("cStatus1 %d\n", cStatus);
	if (cStatus == MI_OK) {
		//memcpy(ucComMF522Buf, pData, 16);
		for (uc = 0; uc < 16; uc++)
			ucComMF522Buf[uc] = *(pData + uc);

		CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);

		cStatus = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &ulLen);
		DEBUG("cStatus2 %d\n", cStatus);
		if ((cStatus != MI_OK) || (ulLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
			cStatus = MI_ERR;

	}


	return cStatus;


}


/*
 * 函数名：PcdRead
 * 描述  ：读取M1卡一块数据
 * 输入  ：u8 ucAddr，块地址
 *         pData，读出的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdRead(u8 ucAddr, u8* pData) {
	char cStatus;
	u8 uc, ucComMF522Buf[MAXRLEN];
	u32 ulLen;
	ucComMF522Buf[0] = PICC_READ;
	ucComMF522Buf[1] = ucAddr;
	CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
	cStatus = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &ulLen);
	if ((cStatus == MI_OK) && (ulLen == 0x90)) {
		for (uc = 0; uc < 16; uc++)
			*(pData + uc) = ucComMF522Buf[uc];
	} else
		cStatus = MI_ERR;
	return cStatus;
}


/*
 * 函数名：PcdHalt
 * 描述  ：命令卡片进入休眠状态
 * 输入  ：无
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdHalt(void) {
	u8 ucComMF522Buf[MAXRLEN];
	u32  ulLen;


	ucComMF522Buf[0] = PICC_HALT;
	ucComMF522Buf[1] = 0;

	CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
	PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &ulLen);

	return MI_OK;

}


/*
 * 函数名：DealWithNewGunma
 * 描述  ：正式处理新滚码
 * 输入  ：新滚码
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */

void DealWithNewGunma(u8 newGunma, u8 oldGunma) {

	IC_NODE_TYPE* pNode = (IC_NODE_TYPE*)&gIcSectorBuf;
	int i;
	uint8_t addr = gGunmaSetting.sector_no * 4; // 所在扇区的地址

	uint8_t blockIndex = 0;
	char result;
	u8   buf[32] = { 0 };
	u8 verify_data[16] = { 0 }; // 用于写后校验

	if (addr == 0) {
		buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_SYSTEM_ERROR;
		buf[1] = newGunma;
		gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 3);
		delay1ms(1000);
		return;
	}
	if (oldGunma == 0) {
		// 没有旧滚码：按gunma_device_id查找，写入新滚码，gunma_old置0
		for (i = 0; i < 12; i++, pNode++) {
			if (pNode->gunma_device_id == gGunmaSetting.gunmaDeviceId || pNode->gunma_device_id == 0) {
				if (pNode->gunma_device_id == 0) {
					pNode->gunma_device_id = gGunmaSetting.gunmaDeviceId;
				}
				pNode->gunma = newGunma;
				pNode->gunma_old = oldGunma;  // =0
				DEBUG("i %d gunma %d, gunma_old %d, gunmadeviceid %d", i, newGunma, oldGunma, pNode->gunma_device_id);
				break;
			}
		}
	} else {
		// 双滚码方案：在IC卡中查找旧滚码(匹配gunma或gunma_old任一即可)
		for (i = 0; i < 12; i++, pNode++) {
			if (pNode->gunma == oldGunma || pNode->gunma_old == oldGunma) {
				pNode->gunma_device_id = gGunmaSetting.gunmaDeviceId;
				// 双滚码轮转: 新滚码→gunma, 当前匹配的旧滚码→gunma_old
				pNode->gunma = newGunma;
				pNode->gunma_old = oldGunma;
				DEBUG("Dual gunma write: new=%d old=%d at idx=%d", newGunma, oldGunma, i);
				break;
			}
		}
	}

	if (i >= 12) {
		// 没找到对应数据，直接返回失败
		if (oldGunma != 0) {
			buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_FUZHI_CARD;
		} else {
			buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_SYSTEM_ERROR;
		}

		buf[1] = newGunma;
		gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 3);
		return;
	}

	blockIndex = i / 4; // 计算需要写入数据的块地址偏移
	addr = addr + blockIndex; // 计算所在块在的地址

	DEBUG("DealWithNewGunma %d %d %d", addr, blockIndex, newGunma);

	// 在 DealWithNewGunma 内部调用写操作前
	PcdHalt(); // 先停掉之前的乱七八糟的状态
	WriteRawRC(CommandReg, PCD_IDLE);
	ClearBitMask(Status2Reg, 0x08); // 彻底关掉上一次 DealWithCard 留下的加密位
	WriteRawRC(FIFOLevelReg, 0x80); // 清空 FIFO

	// --- 第一步：物理唤醒 (针对 Stat1 0x31) ---

		  /*
		PcdAntennaOff();
		delay1ms(50);
		PcdAntennaOn();
		delay1ms(50);
		Wdt_Feed();
			*/
			// --- 第二步：清理寄存器 ---
#if 0	
	WriteRawRC(CommandReg, PCD_IDLE);
	ClearBitMask(Status2Reg, 0x08);
	WriteRawRC(BitFramingReg, 0x07); // 寻卡模式				
	DEBUG("pchalt 2");
	Wdt_Feed();
	if (PcdRequest(PICC_REQALL, g_ucTempbuf) == MI_OK) {
		DEBUG("pchalt 3");
		if (PcdAnticoll(active_uid) == MI_OK) {
			DEBUG("pchalt 4");
			if (PcdSelect(active_uid) != MI_ERR) {
				DEBUG("pchalt 5");
				// 重新认证目标地址
				if (PcdAuthState(PICC_AUTHENT1A, addr, DefaultKey, active_uid) == MI_OK) {
					// 此时再调用 PcdWrite
					Wdt_Feed();
					result = PcdWrite(addr, (u8*)(&gIcSectorBuf.block[blockIndex]));
					DEBUG("PcdWrite resut %d addr %d blockIndex %d\n ", result, addr, blockIndex);

					if (result == MI_OK) {
						buf[0] = GUNMA_IC_VERIFY_RESULT_SUCCESS;
						buf[1] = newGunma;
						gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 3);
					}
				}
			}
		}
	} else {
		u8 err = ReadRawRC(ErrorReg);
		u8 st1 = ReadRawRC(Status1Reg);
		DEBUG("Request Fail! Err:0x%02X, Stat1:0x%02X\n", err, st1);
	}
#else
		// 改进后的逻辑：增加内部重试和强制复位
	int auth_retry = 15;
	while (auth_retry--) {
		Wdt_Feed();
		//WriteRawRC(CommandReg, PCD_IDLE);
		//ClearBitMask(Status2Reg, 0x08); // 关键：每一轮认证前都必须关掉加密位

		if (PcdRequest(PICC_REQALL, g_ucTempbuf) == MI_OK) {
			if (PcdAnticoll(active_uid) == MI_OK) {
				if (PcdSelect(active_uid) != MI_ERR) {
					if (PcdAuthState(PICC_AUTHENT1A, addr, DefaultKey, active_uid) == MI_OK) {
						// 授权成功，执行写入（带内部重试）
						int write_retry = 3;
						int write_ok_but_mismatch = 0;
						while (write_retry--) {
							Wdt_Feed();
							result = PcdWrite(addr, (u8*)(&gIcSectorBuf.block[blockIndex]));

							if (result == MI_OK) {
								// --- 原子性校验：写完立刻读回来比对 ---
								delay1ms(5);
								if (PcdRead(addr, verify_data) == MI_OK) {
									if (memcmp(verify_data, &gIcSectorBuf.block[blockIndex], 16) == 0) {
										// 物理写入完全确认，发送成功协议
										buf[0] = GUNMA_IC_VERIFY_RESULT_SUCCESS;
										buf[1] = newGunma;
										memcpy(buf + 2, active_uid, 4);
										gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 7);
										DEBUG("Gunma Write & Verify Success! Addr:%d\n", addr);

										PcdHalt();
										//成功刷卡之后，延迟100ms，防止卡片被再次刷卡
										delay1ms(200);
										Wdt_Feed();
										delay1ms(200);
										Wdt_Feed();
										return; // 成功后直接跳出函数
									}
									// 写入成功但数据不一致，标记并继续重试
									write_ok_but_mismatch = 1;
									DEBUG("Write OK but mismatch, retry:%d\n", write_retry);
								} else {
									write_ok_but_mismatch = 2;
								}
								break;
							}
						}

						// 写重试耗尽：如果写入成功过但数据始终不一致，读卡内实际滚码作为结果
						if (write_ok_but_mismatch) {
							IC_NODE_TYPE* pActualNode = (IC_NODE_TYPE*)verify_data;
							uint8_t actualGunma = 0;
							buf[0] = GUNMA_IC_VERIFY_RESULT_SUCCESS;
							buf[1] = actualGunma;
							memcpy(buf + 2, active_uid, 4);
							gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 7);
							DEBUG("Gunma Write Mismatch, Use Card Gunma:%d (expected:%d)\n", actualGunma, newGunma);
							PcdHalt();
							return;

						}
						Wdt_Feed();
					}
				}
			}
		}
		// 如果失败，强制关一下天线再开（场复位），给卡片喘息机会
		PcdAntennaOff();
		delay1ms(10);
		PcdAntennaOn();
		delay1ms(10);
	}
	// 4. 彻底失败处理
	u8 err = ReadRawRC(ErrorReg);
	u8 st1 = ReadRawRC(Status1Reg);
	DEBUG("Write Final Fail! Err:0x%02X, Stat1:0x%02X\n", err, st1);

	buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_GUNMA_WRITE_FAIL;
	buf[1] = newGunma;
	gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 3);

#endif


}

/*
 * 函数名：DealWithCard
 * 描述  ：正式处理卡
 * 输入  ：卡号
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char DealWithCard(u8* UID) {
	u8 result = 0;
	u8 addr = gGunmaSetting.sector_no == 0 ? 8 : gGunmaSetting.sector_no * 4;
	IC_NODE_TYPE* icNode;

	u8 gunma = 0;
	u8 buf[16] = { 0 };
	u32 activeId = 0;
	int i;


	if (gGunmaSetting.gunmaDeviceId == 0) {
		memcpy(buf, UID, 4);
		memcpy(buf + 4, &gunma, 1);
		gunmaProtocol_Send(GUNMA_CMD_RECEIVE_CARDNO, buf, 6);
		gGunamICState = GUNMA_IC_STATE_IDLE;
		return MI_OK;
	}

	if (addr == 0) {
		return MI_ERR;
	}

	gGunamICState = GUNMA_IC_STATE_SENDING_IC_CARD;
	DEBUG("%02x %02x %02x %02x %d\n", UID[0], UID[1], UID[2], UID[3], addr);

	result = PcdAuthState(PICC_AUTHENT1A, addr, DefaultKey, UID);
	Wdt_Feed();

	if (result != MI_OK) {
		/* 认证失败：卡密钥非默认 0xFFFFFFFF（或密钥 A 不可用）。
		 * 原先直接 return，不向 HC32 上报，表现为刷卡完全无反应；
		 * 现上报非法卡事件，使主机端能给出报警提示。 */
		DEBUG("auth fail");
		buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD;
		buf[1] = INVALID_CARD_AUTH_FAIL;
		gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
		return MI_ERR;
	}

	memset(&gIcSectorBuf, 0, sizeof(gIcSectorBuf));
	for (i = 0; i < 3; i++) {
		int j = 0;
		do {
			result = PcdRead(addr + i, (u8*)(gIcSectorBuf.block + i));
			j++;
			if (j > 3) {
				buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_GUNMA_WRITE_FAIL;
				buf[1] = INVALID_CARD_7_UID;
				gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
				return MI_ERR;
			}
		} while (result != MI_OK);
		Wdt_Feed();
	}

	IC_NODE_TYPE* pNode = (IC_NODE_TYPE*)&gIcSectorBuf;


	for (i = 0; i < 12; i++, pNode++) {
		if (pNode->gunma_device_id == gGunmaSetting.gunmaDeviceId) {
			break;
		}
	}

	if (i < 12) {
		// 双滚码方案：发送2字节滚码 [gunma, gunma_old]
		u8 gunma_buf[2];
		gunma_buf[0] = pNode->gunma;
		gunma_buf[1] = pNode->gunma_old;
		memcpy(buf, UID, 4);
		memcpy(buf + 4, gunma_buf, 2);
		gunmaProtocol_Send(GUNMA_CMD_RECEIVE_CARDNO, buf, 6);
	} else {
		memcpy(buf, UID, 4);
		memset(buf + 4, 0, 2);
		gunmaProtocol_Send(GUNMA_CMD_RECEIVE_CARDNO, buf, 6);
	}
	gGunamICState = GUNMA_IC_STATE_SENDING_IC_CARD;
	return MI_OK;

}


void IC_CMT(u8* UID, u8* KEY, u8 RW, u8* Dat) {
	u8 ucArray_ID[4] = { 0 };//先后存放IC卡的类型和UID(IC卡序列号)
	PcdRequest(0x52, ucArray_ID);//寻卡
	PcdAnticoll(ucArray_ID);//防冲撞
	PcdSelect(UID);//选定卡
	PcdAuthState(0x60, 0x10, KEY, UID);//校验
	if (RW)//读写选择，1是读，0是写
		PcdRead(0, Dat);
	else
		PcdWrite(0x10, Dat);
	PcdHalt();
}


void CS(void) {
	char status;
	uint8_t buf[64];
	uint32_t current_tick;
	int result;
	// 1. 暴力复位环境

	status = PcdRequest(PICC_REQALL, g_ucTempbuf);

	if (status == MI_OK) {
		// ATQA的值为0x44，7位id，直接返回
		if ((g_ucTempbuf[1] & 0x40) != 0) {
			buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD;
			buf[1] = INVALID_CARD_7_UID;
			gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
			return;
		}
		u8* SelectedSnr = g_ucTempbuf + 2;

		if (PcdAnticoll(SelectedSnr) == MI_OK) {

			status = PcdSelect(SelectedSnr);
			result = memcmp(active_uid, SelectedSnr, 4);
			current_tick = Get_SysTick();
			// 处理溢出的情况
			if (current_tick < wait_start_tick) {
				wait_start_tick = current_tick;
			}
			if (status == SAK_CPU_NFC) {
				// 仅仅在不是同一个卡号，或者刷卡时间大于2秒，进行播报，否则仅仅返回
				if (result != 0 || (current_tick - wait_start_tick) > 2000) {
					memcpy(active_uid, SelectedSnr, 4);

					wait_start_tick = current_tick;
					buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD;
					buf[1] = INVALID_CARD_NFC_CPU;
					gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
					PcdHalt();
				}
				return;
			}

			// 做卡的重复检测
			/*
			if((result == 0 || result == -1) && (current_tick - wait_start_tick) < 4000){
				PcdHalt();
				return;
			} */
			// 禁止卡刷太快
			DEBUG("%d, %d\n", current_tick - wait_start_tick, gGunamICState);
			if (current_tick - wait_start_tick < 1000) {
				return;
			}
			if ((current_tick - wait_start_tick) < 3000 && gGunamICState != GUNMA_IC_STATE_IDLE) {

				PcdHalt();
				Wdt_Feed();

				return;
			}
			gGunamICState = GUNMA_IC_STATE_SELECT_CARD;  // 进入选卡阶段
			memcpy(active_uid, SelectedSnr, 4);
			wait_start_tick = current_tick;
			DEBUG("1:%d, %d\n", current_tick - wait_start_tick, gGunamICState);
			// DEBUG("%02X %02X %02X %02X", SelectedSnr[0],SelectedSnr[1],SelectedSnr[2],SelectedSnr[3]);
		   // 第一步：后门检测 (0x40/0x43)
		   // --- 第一阶段：克隆检测 ---
			uint8_t isClone = 0;

			if (Check_By_Writing_Block0(SelectedSnr)) {

				buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD;
				buf[1] = INVALID_CARD_WRITE_0_SECTOR;
				gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
				gGunamICState = GUNMA_IC_STATE_IDLE;
				isClone = 1;
				DEBUG("Detected: CUID Card (Writability)\r\n");
			} else if (Check_Is_CloneCard()) {

				Safe_Write_UID_Backdoor();
				buf[0] = GUNMA_IC_VERIFY_RESULT_FAIL_INVALID_CARD;
				buf[1] = INVALID_CARD_BACK_DOOR;
				gunmaProtocol_Send(GUNMA_CMD_IC_VERIFY_RESULT, buf, 2);
				gGunamICState = GUNMA_IC_STATE_IDLE;
				isClone = 2;
				DEBUG("Detected: UID/Magic Card (Backdoor) %d %d\r\n", current_tick, result);
			}

			if (isClone > 0) {
				// 发送克隆卡报信

				PcdHalt();
				delay1ms(10); // 等待处理完成再处理下一个指令
				return;
			}
			// --- 第二阶段：准备正式读卡 (关键修复点) ---
			// 经过上面的探测，卡片状态已经乱了，必须重新激活
			// 核心修复：扇区0加密卡在Check_By_Writing_Block0中认证失败后，
			// M1卡进入死锁状态，仅靠PcdHalt无法恢复，必须做RF场复位（天线关→开）
			PcdHalt();
			delay1ms(10);
			// RF场复位：模拟卡片离开再靠近，强制卡片重置状态机
			PcdAntennaOff();
			delay1ms(10);
			PcdAntennaOn();
			delay1ms(10);
			Wdt_Feed();

			Wdt_Feed();
			int auth_retry = 5;
			while (auth_retry--) {
				WriteRawRC(CommandReg, PCD_IDLE);
				ClearBitMask(Status2Reg, 0x08); // 关键：每一轮认证前都必须关掉加密位
				status = PcdRequest(PICC_REQALL, g_ucTempbuf);
				if (status == MI_OK) {
					if (PcdAnticoll(SelectedSnr) == MI_OK) {
						if (PcdSelect(SelectedSnr) != MI_ERR) {
							DEBUG("#2#");
							// 此时卡片回到了标准状态，可以安全认证了
							if (DealWithCard(SelectedSnr) == MI_OK) {
								DEBUG("#3#");
								Wdt_Feed();
								return;
							}
							Wdt_Feed();

						}
					}
				}
				// 每一轮重试也做RF场复位
				PcdAntennaOff();
				delay1ms(10);
				PcdAntennaOn();
				delay1ms(10);
				Wdt_Feed();
			}
			// 如果没有操作成功，状态设置为idle，等待下一个卡号
			gGunamICState = GUNMA_IC_STATE_IDLE;

		}
	}
}