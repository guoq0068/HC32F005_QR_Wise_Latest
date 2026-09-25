#ifndef __RC522_FUNCTION_H
#define	__RC522_FUNCTION_H

#include "string.h"
#include "rc522_config.h"
#include <stdint.h>
#include <ddl.h>
#include "hc32f005.h"
#include "core_cm0plus.h"
//#include "delay.h"
//#include "USART.h"
//#include "key.h"
//#include "led.h"
//#include "beep.h"

#ifndef __nop
  #define __nop() __asm("NOP")
#endif

extern int exti_t;
extern float buffer;

typedef uint8_t u8;
typedef uint32_t u32;

#define   macRC522_DELAY()  delay10us(1)
#define          macDummy_Data              0x00


void             PcdReset                   ( void );                       //复位
void             M500PcdConfigISOType       ( u8 type );                    //工作方式
char             PcdRequest                 ( u8 req_code, u8 * pTagType ); //寻卡
char             PcdAnticoll                ( u8 * pSnr);                   //读卡号
void                    SysTick_Init                            ( void );
void                    TimingDelay_Decrement                   ( void );
char PcdRead ( u8 ucAddr, u8 * pData );//读卡
char PcdWrite ( u8 ucAddr, u8 * pData );//写卡
char PcdAuthState ( u8 ucAuth_mode, u8 ucAddr, u8 * pKey, u8 * pSnr );//验证卡片密码
char PcdSelect ( u8 * pSnr );//选卡
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);//扣款和充值
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);//备份钱包
char PcdHalt( void );//命令卡休眠
void CZ(void);
void Clear_Money(void);
void XF(void);
void CS(void);
#endif /* __RC522_FUNCTION_H */

