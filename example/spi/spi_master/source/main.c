/******************************************************************************
* Copyright (C) 2019, Xiaohua Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Xiaohua Semiconductor Co.,Ltd ("XHSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with XHSC
* components. This software is licensed by XHSC to be adapted only
* for use in systems utilizing XHSC components. XHSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. XHSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* XHSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* XHSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2019-04-23  1.0   First version for Device Driver Library of Module.
 **
 ******************************************************************************/

 /******************************************************************************
  * Include files
  ******************************************************************************/
#include "ddl.h"
#include "spi.h"
#include "gpio.h"
#include "rc522_function.h"
#include "uart.h"
#include "sysctrl.h"
#include "bt.h"
#include "lpm.h"
#include "flash.h"
#include "wdt.h"
#include "guic_gunma.h"




  /******************************************************************************
   * Local pre-processor symbols/macros ('#define')
   ******************************************************************************/

#define  BTU_2400  9950
#define  BTU_2400_1_5  14950
#define  BTU_4800  4970
#define  BTU_4800_1_5  7470
   // 假设系统时钟 PCLK = 24MHz (若为8MHz请重新计算)
   // 9600波特率：1.0位宽=2500个脉冲，1.5位宽=3750个脉冲
#define T0_1_0_BIT    (65536 - BTU_2400) // 略微加快采样  2430 10000 5000
#define T0_1_5_BIT    (65536 - BTU_2400_1_5) // 3400 15000  7500

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint8_t  g_u8RxData = 0;
volatile uint8_t  g_u8RxBitCnt = 0;
volatile boolean_t g_bRxBusy = FALSE;
volatile boolean_t g_bDataReady = FALSE;

uint8_t u8RxData[8] = { 0x00 };
uint8_t ucnt = 0;
uint8_t u8RxFlg = 0;
uint8_t CheckFlg = 0;

uint8_t temp_buf = 0;
boolean_t has_data = FALSE;

#define RX_BUF_SIZE  64        // 缓冲区大小
uint8_t  g_u8RxBuf[RX_BUF_SIZE]; // 缓冲区数组
volatile uint8_t  g_u8NextW = 0; // 写指针
volatile uint8_t  g_u8NextR = 0; // 读指针
volatile uint32_t g_u32ResetCount = 0;  // 复位busy的计数器


volatile uint32_t u32SystemTick = 0; // 全局毫秒计数器


__IO uint8_t tx_cnt, rx_cnt;

void App_UartPortCfg(void);
void App_UartInit(void);
void App_ClkInit(void);
void uart_data_loop(void);
void SoftUart_Init(void);
void SoftUart_SendByte(uint8_t u8Data);
void gunmaDataProcess(uint8_t rxData);
void Timer0_Init(void);
void RC522_Init(void);
/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

 /******************************************************************************
  * Local function prototypes ('static')
  ******************************************************************************/
static void App_GpioInit(void);
static void App_SPIInit(void);
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/


 /******************************************************************************
  * Local pre-processor symbols/macros ('#define')
  ******************************************************************************/

  /*****************************************************************************
   * Function implementation - global ('extern') and local ('static')
   ******************************************************************************/
static void App_PortInit(void);

///<App_WdtInit 看门狗函数
static void App_WdtInit(void) {
  ///< 开启WDT外设时钟
  Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt, TRUE);
  ///< WDT 初始化并立即启动
  Wdt_Init(WdtResetEn, WdtT820ms);
  Wdt_Start();
}



// 封装成你需要的获取函数
uint32_t Get_SysTick(void) {
  return u32SystemTick;
}

/**
******************************************************************************
    ** \brief  主函数
    **
  ** @param  无
    ** \retval 无
    **
******************************************************************************/
int32_t main(void) {
  uint16_t tmp;
  volatile uint8_t tmp1;
  tx_cnt = 0;
  tmp = 0;

  u8RxFlg = 0;
  ucnt = 0;

  /// 时钟初始化，如果不初始化，那么会出现串口乱码的问题
  App_ClkInit();
  ///< UART 端口初始化
  //App_PortInit();

  ///< UART 初始化
  //App_UartInit();

  Timer0_Init();
  SoftUart_Init();



  // 2. 稍微延时一下，等电平稳定，防止复位瞬间的乱码
  /*
  for(volatile int i=0; i<100000; i++);
  delay1ms(1000);

  SoftUart_SendByte(0x55);

  SoftUart_SendByte(0x55);

  SoftUart_SendByte(0x55);

  SoftUart_SendByte(0x55);

  SoftUart_SendByte(0x55);

  SoftUart_SendByte(0x57);
  printf("\r\n--- UART STABLE NOW ---\r\n");


  SoftUart_SendByte(0x55); */
  RC522_Init();
  delay1ms(500);

  PcdReset();
  M500PcdConfigISOType('A');//设置工作方式
  App_WdtInit();

  ///< 确保初始化正确执行后方能进行FLASH编程操作，FLASH初始化（编程时间）
  gunmaInit();

  __enable_irq(); // <--- 确保这一行在 while(1) 之前！

  while (1) {
    CS();
    uart_data_loop();
    Wdt_Feed();
    //
  };
}


void Timer0_Init(void) {
  stc_bt_cfg_t stcCfg;

  Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE); // 开启时钟门控

  /*
    stcConfig.enMD = BtMode2;          // 自动重装载模式
    stcConfig.enCT   = BtTimer;          // 定时器模式
    stcConfig.enPRS  = BtPCLKDiv1;       // 不分频
    */

  stcCfg.enGateP = BtPositive;
  stcCfg.enGate = BtGateDisable;
  stcCfg.enPRS = BtPCLKDiv8;
  stcCfg.enTog = BtTogDisable;
  stcCfg.enCT = BtTimer;
  stcCfg.enMD = BtMode2;
  Bt_Init(TIM0, &stcCfg);           // 初始化



  Bt_ClearIntFlag(TIM0);               // 清除中断标志
  Bt_EnableIrq(TIM0);                  // 使能定时器中断;            
  EnableNvic(TIM0_IRQn, IrqLevel3, TRUE); // 使能 NVIC 中断

  // 假设 PCLK 是 24MHz，计算 1ms 的重装载值
// 关键：必须同时设置重装载值 (ARR)
// 24000 对应 1ms @ 24MHz
  uint16_t u16Period = 65536 - 3500;
  Bt_ARRSet(TIM0, u16Period);
  Bt_Cnt16Set(TIM0, u16Period);
  Bt_Run(TIM0);                        // 启动定时器
}


void Tim0_IRQHandler(void) {
  if (TRUE == Bt_GetIntFlag(TIM0)) {
    u32SystemTick++;                 // 这里的变量记得加 volatile
    Bt_ClearIntFlag(TIM0);
  }
}

void SoftUart_Init(void) {
  stc_gpio_cfg_t stcGpioCfg;
  stc_bt_cfg_t   stcCfg;

  // 1. 开启外设时钟
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
  Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

  // 2. 配置 P35 (RX) - 输入 + 上拉 + 下降沿中断
  DDL_ZERO_STRUCT(stcGpioCfg);
  stcGpioCfg.enDir = GpioDirIn;
  stcGpioCfg.enPu = GpioPuEnable;  // 开启内部上拉，非常关键
  //stcGpioCfg.enIrq = GpioIrqFalling;
  Gpio_Init(GpioPort3, GpioPin5, &stcGpioCfg);

  // 3. 配置 P36 (TX) - 推挽输出
  stcGpioCfg.enDir = GpioDirOut;
  stcGpioCfg.enPu = GpioPuDisable;
  //stcGpioCfg.enIrq = GpioIrqDisable;
  Gpio_Init(GpioPort3, GpioPin6, &stcGpioCfg);
  Gpio_WriteOutputIO(GpioPort3, GpioPin6, TRUE); // 空闲态为高

  // 4. 配置 Timer0 (Base Timer)
  //打开BT外设时钟
  Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

  // 3. 配置 TIM2 (专门服务软件串口)
  stcCfg.enGateP = BtPositive;
  stcCfg.enGate = BtGateDisable;
  stcCfg.enPRS = BtPCLKDiv1; // 24MHz 不分频
  stcCfg.enTog = BtTogDisable;
  stcCfg.enCT = BtTimer;
  stcCfg.enMD = BtMode2;    // 16位自动重装模式
  Bt_Init(TIM2, &stcCfg);      // 使用 TIM2

  //TIM2中断使能
  Bt_ClearIntFlag(TIM2);
  Bt_EnableIrq(TIM2);
  EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);

  // 5. 中断使能
  //EnableNvic(PORT3_IRQn, IrqLevel3, TRUE);
  //EnableNvic(TIM0_IRQn, IrqLevel2, TRUE);
  //Bt_EnableIrq(TIM0);

  // 必须明确开启 P35 的下降沿中断触发
  // --- 关键：必须手动开启引脚中断 ---
  Gpio_ClearIrq(GpioPort3, GpioPin5);
  Gpio_EnableIrq(GpioPort3, GpioPin5, GpioIrqFalling);
  EnableNvic(PORT3_IRQn, IrqLevel1, TRUE);
}


// 处理 P35 起始位触发
void Port3_IRQHandler(void) {

  if (TRUE == Gpio_GetIrqStatus(GpioPort3, GpioPin5)) {
    //printf("p3\n");
    if (!g_bRxBusy) {
      Bt_Stop(TIM2); // 强制停一下，确保万无一失
      Bt_ClearIntFlag(TIM2);

      g_bRxBusy = TRUE;
      g_u8RxBitCnt = 0;
      g_u8RxData = 0;
      g_u32ResetCount = 1000;

      // 设为 1.5 倍位宽延时，对准第一个数据位中心
      Bt_Cnt16Set(TIM2, T0_1_5_BIT);
      Bt_Run(TIM2);

      // 接收期间暂时关闭引脚中断
      Gpio_DisableIrq(GpioPort3, GpioPin5, GpioIrqFalling);
    }
    Gpio_ClearIrq(GpioPort3, GpioPin5);
  }
}

// 处理 Timer0 采样触发
void Tim2_IRQHandler(void) {

  if (TRUE == Bt_GetIntFlag(TIM2)) {
    Bt_ClearIntFlag(TIM2); // 必须第一时间清除，否则会影响下次中断进入时间
    if (g_bRxBusy == TRUE) // 使用你定义的全局布尔变量
    {
      if (g_u8RxBitCnt < 8) {
        // 先采样，再移位
        g_u8RxData >>= 1;
        if (TRUE == Gpio_GetInputIO(GpioPort3, GpioPin5)) {
          g_u8RxData |= 0x80;
        }
        g_u8RxBitCnt++;

        // 设置下一次采样的定时（1.0个位宽）
        Bt_Cnt16Set(TIM2, T0_1_0_BIT);
      } else {
        Bt_Stop(TIM2);
        Bt_ClearIntFlag(TIM2); // 务必清理定时器标志
        g_bRxBusy = FALSE;
        g_u32ResetCount = 0;
        // 将收到的数据存入缓冲区
        g_u8RxBuf[g_u8NextW] = g_u8RxData;
        g_u8NextW = (g_u8NextW + 1) % RX_BUF_SIZE; // 循环写入

        // 关键：重置计数寄存器，防止残留值干扰下一次起始位触发
        Bt_Cnt16Set(TIM2, T0_1_0_BIT);
        // 重新启用前，先清理掉刚才可能积压的干扰中断
        Gpio_ClearIrq(GpioPort3, GpioPin5);
        Gpio_EnableIrq(GpioPort3, GpioPin5, GpioIrqFalling);
      }
    }
  } else {
  }
}


void uart_data_loop() {

  // 如果读写指针不等，说明缓冲区里有没读完的数据
  if (g_u32ResetCount > 1) {
    g_u32ResetCount--;
  } else {
    if (g_u32ResetCount == 1) {
      g_bRxBusy = FALSE;
      g_u32ResetCount = 0;
      Bt_Cnt16Set(TIM2, T0_1_0_BIT);
      // 重新启用前，先清理掉刚才可能积压的干扰中断
      Gpio_ClearIrq(GpioPort3, GpioPin5);
      Gpio_EnableIrq(GpioPort3, GpioPin5, GpioIrqFalling);
    }
  }
  if (g_u8NextR != g_u8NextW) {

    if (g_bRxBusy) return;

    __disable_irq();
    uint8_t data_to_send = g_u8RxBuf[g_u8NextR];
    //printf(" %02x ", data_to_send);
    g_u8NextR = (g_u8NextR + 1) % RX_BUF_SIZE; // 移动读指针
    __enable_irq();
    // 执行发送
    //SoftUart_SendByte(data_to_send);
    gunmaDataProcess(data_to_send);
    // 关键：发完后喂狗
    Wdt_Feed();

    // 给上位机和硬件一点时间切换状态，防止连续发送阻塞 RX
    for (volatile int i = 0; i < 500; i++);
  }

  /*
  uint8_t dataTemp = 0;

  if(g_bDataReady){

    dataTemp = g_u8RxData;
    g_bDataReady = FALSE;
    SoftUart_SendByte(g_u8RxData);
    gunmaDataProcess(dataTemp);
    Wdt_Feed();
  }*/
}

void SoftUart_SendByte(uint8_t u8Data) {
  // 24MHz / 9600 = 2500 周期
  // 4800 波特率： 5000
  // 2400 波特率：10000
  uint16_t u16Period = 65536 - 10000; // 2500 9920   

  __disable_irq(); // 必须关中断，保证时序是纯净的
  // 1. 发送前加锁：禁止 TIM2 中断，防止干扰发送循环
  Bt_Stop(TIM0);
  Bt_DisableIrq(TIM2);

  // --- 起始位 ---
  Gpio_WriteOutputIO(GpioPort3, GpioPin6, FALSE);
  Bt_Cnt16Set(TIM2, u16Period);
  Bt_Run(TIM2);
  while (Bt_GetIntFlag(TIM2) == FALSE); // 硬件死等，不受软件运行速度影响
  Bt_ClearIntFlag(TIM2);

  // --- 数据位 (8位) ---
  for (uint8_t i = 0; i < 8; i++) {
    Gpio_WriteOutputIO(GpioPort3, GpioPin6, (u8Data & 0x01));
    u8Data >>= 1;
    Bt_Cnt16Set(TIM2, u16Period);
    while (Bt_GetIntFlag(TIM2) == FALSE);
    Bt_ClearIntFlag(TIM2);
  }

  // --- 停止位 ---
  Gpio_WriteOutputIO(GpioPort3, GpioPin6, TRUE);
  Bt_Cnt16Set(TIM2, u16Period);
  while (Bt_GetIntFlag(TIM2) == FALSE);
  Bt_ClearIntFlag(TIM2);

  Bt_Cnt16Set(TIM2, u16Period); // 第二个停止位周期
  while (Bt_GetIntFlag(TIM2) == FALSE);
  Bt_ClearIntFlag(TIM2);

  Bt_Stop(TIM2); // 发完关闭
  Bt_Run(TIM0);
  __enable_irq();
  // 2. 发送完解锁：重新允许 TIM2 中断，准备接收数据
  Bt_ClearIntFlag(TIM2); // 清除发送留下的标志位
  Bt_EnableIrq(TIM2);    // 重新开启中断使能
  //delay10us(10);

}



void SPI1_Init(void) {

  stc_spi_cfg_t  SpiInitStruct;

  Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi, TRUE);

  //SPI0模块配置：主机
  SpiInitStruct.enSpiMode = SpiMskMaster;   //配置位主机模式
  SpiInitStruct.enPclkDiv = SpiClkMskDiv4;  //波特率：fsys/128
  SpiInitStruct.enCPHA = SpiMskCphafirst;//第一边沿采样
  SpiInitStruct.enCPOL = SpiMskcpollow;  //极性为高
  Spi_Init(M0P_SPI, &SpiInitStruct);


}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void) {
  stc_gpio_cfg_t GpioInitStruct;
  DDL_ZERO_STRUCT(GpioInitStruct);

  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

  //SPI0引脚配置:主机
  GpioInitStruct.enDrv = GpioDrvH;
  GpioInitStruct.enDir = GpioDirOut;

  Gpio_Init(EVB_SPI_MOSI_PORT, EVB_SPI_MOSI_PIN, &GpioInitStruct);
  Gpio_SetAfMode(EVB_SPI_MOSI_PORT, EVB_SPI_MOSI_PIN, GpioAf5);        //配置引脚SPI_MOSI

  Gpio_Init(EVB_SPI_CS_PORT, EVB_SPI_CS_PIN, &GpioInitStruct);
  Gpio_SetAfMode(EVB_SPI_CS_PORT, EVB_SPI_CS_PIN, GpioAf2);         //配置引脚SPI_CS

  Gpio_Init(EVB_SPI_SCK_PORT, EVB_SPI_SCK_PIN, &GpioInitStruct);
  Gpio_SetAfMode(EVB_SPI_SCK_PORT, EVB_SPI_SCK_PIN, GpioAf1);         //配置引脚SPI_SCK

  Gpio_Init(GpioPort3, GpioPin3, &GpioInitStruct);
  //Gpio_WriteOutputIO(GpioPort3, GpioPin3, TRUE);

  GpioInitStruct.enDir = GpioDirIn;
  Gpio_Init(EVB_SPI_MISO_PORT, EVB_SPI_MISO_PIN, &GpioInitStruct);
  Gpio_SetAfMode(EVB_SPI_MISO_PORT, EVB_SPI_MISO_PIN, GpioAf5);         //配置引脚SPI_MISO



  ///< 端口方向配置->输入
  GpioInitStruct.enDir = GpioDirIn;
  ///< 端口驱动能力配置->高驱动能力
  GpioInitStruct.enDrv = GpioDrvL;
  ///< 端口上下拉配置->无
  GpioInitStruct.enPu = GpioPuDisable;
  GpioInitStruct.enPd = GpioPdDisable;
  ///< 端口开漏输出配置->开漏输出关闭
  GpioInitStruct.enOD = GpioOdDisable;

  ///< GPIO IO USER KEY初始化
  Gpio_Init(STK_USER_PORT, STK_USER_PIN, &GpioInitStruct);


  //板上LED
  GpioInitStruct.enDrv = GpioDrvH;
  GpioInitStruct.enDir = GpioDirOut;
  Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);
  Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);     //输出高，熄灭LED
}

/**
 ******************************************************************************
 ** \brief  初始化SPI
 **
 ** \return 无
 ******************************************************************************/
static void App_SPIInit(void) {
  stc_spi_cfg_t  SpiInitStruct;

  Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi, TRUE);

  //SPI0模块配置：主机
  SpiInitStruct.enSpiMode = SpiMskMaster;   //配置位主机模式
  SpiInitStruct.enPclkDiv = SpiClkMskDiv8;  //波特率：fsys/128
  SpiInitStruct.enCPHA = SpiMskCphafirst;//第一边沿采样
  SpiInitStruct.enCPOL = SpiMskcpollow;  //极性为低
  Spi_Init(M0P_SPI, &SpiInitStruct);


}

void App_ClkInit(void) {
  Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
  Sysctrl_SysClkSwitch(SysctrlClkRCL);
  Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);       //配置为外部24MHz时钟, GPIO输出24mhz主频
  Sysctrl_SysClkSwitch(SysctrlClkRCH);
  Sysctrl_ClkSourceEnable(SysctrlClkRCL, FALSE);

}



void App_UartPortCfg(void) {
  stc_gpio_cfg_t stcGpioCfg;

  DDL_ZERO_STRUCT(stcGpioCfg);

  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //使能GPIO模块时钟

  ///<TX
  stcGpioCfg.enDir = GpioDirOut;
  Gpio_Init(GpioPort3, GpioPin5, &stcGpioCfg);
  Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);          //配置P35 端口为URART1_TX

  ///<RX
  stcGpioCfg.enDir = GpioDirIn;
  Gpio_Init(GpioPort3, GpioPin6, &stcGpioCfg);
  Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);          //配置P36 端口为URART1_RX
}

///< UART1 中断服务函数
void Uart1_IRQHandler(void) {
  uint8_t temp;
  if (TRUE == Uart_GetStatus(M0P_UART1, UartRC)) {
    Uart_ClrStatus(M0P_UART1, UartRC);

    temp = Uart_ReceiveData(M0P_UART1);
    //u8RxData[1] = temp;
    //u8RxFlg = 1;

    if (temp == 0xd || ucnt == 7) {
      u8RxFlg = 1;
    } else {
      u8RxData[ucnt] = temp;
      ucnt++;
    }

  }

}

//串口引脚配置
static void App_PortInit(void) {
  stc_gpio_cfg_t stcGpioCfg;

  DDL_ZERO_STRUCT(stcGpioCfg);

  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //使能GPIO模块时钟

  ///<TX
  stcGpioCfg.enDir = GpioDirOut;
  Gpio_Init(GpioPort3, GpioPin5, &stcGpioCfg);
  Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);          //配置P35 端口为URART1_TX

  ///<RX
  stcGpioCfg.enDir = GpioDirIn;
  Gpio_Init(GpioPort3, GpioPin6, &stcGpioCfg);
  Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);          //配置P36 端口为URART1_RX

}

static void _UartBaudCfg(void) {
  uint16_t timer = 0;

  stc_uart_baud_cfg_t stcBaud;
  stc_bt_cfg_t stcBtCfg;

  //SystemCoreClockUpdate(); // 必须调用！强制同步当前主频变量

  DDL_ZERO_STRUCT(stcBaud);
  DDL_ZERO_STRUCT(stcBtCfg);

  //外设时钟使能
  Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);//模式0/2可以不使能
  Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1, TRUE);

  stcBaud.bDbaud = 0u;//双倍波特率功能
  stcBaud.u32Baud = 9600u;//更新波特率位置
  stcBaud.enMode = UartMode1; //计算波特率需要模式参数
  stcBaud.u32Pclk = Sysctrl_GetPClkFreq(); // Sysctrl_GetPClkFreq(); //获取PCLK
  timer = Uart_SetBaudRate(M0P_UART1, &stcBaud);

  stcBtCfg.enMD = BtMode2;
  stcBtCfg.enCT = BtTimer;
  Bt_Init(TIM1, &stcBtCfg);//调用basetimer1设置函数产生波特率
  Bt_ARRSet(TIM1, timer);
  Bt_Cnt16Set(TIM1, timer);
  Bt_Run(TIM1);

}





void App_UartInit(void) {
  stc_uart_cfg_t  stcCfg;

  _UartBaudCfg();

  stcCfg.enRunMode = UartMode1;//测试项，更改此处来转换4种模式测试
  Uart_Init(M0P_UART1, &stcCfg);

  ///< UART中断配置
  Uart_EnableIrq(M0P_UART1, UartRxIrq);
  Uart_ClrStatus(M0P_UART1, UartRC);
  EnableNvic(UART1_IRQn, IrqLevel3, TRUE);

}


// 重定向 fputc 到 UART0
int fputc(int ch, FILE* f) {

  //Uart_SendDataPoll(M0P_UART1,ch);
  SoftUart_SendByte(ch);
  // 9600波特率一位大约104us，这里延时约150us
  for (volatile int i = 0; i < 1000; i++);
  return ch;
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


