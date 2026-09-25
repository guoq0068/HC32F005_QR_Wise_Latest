#include "rc522_config.h"

static void             RC522_SPI_Config             ( void );

void SPI1_Init(void);

void RC522_Init ( void )
{
	RC522_SPI_Config ();
	
	SPI1_Init();
	
	while(1 == Gpio_GetInputIO(EVB_SPI_CS_PORT,EVB_SPI_CS_PIN));//等待片选信号生效
	
	macRC522_Reset_Disable();
	
	delay1ms(100);
	
	macRC522_Reset_Enable();
	
	delay1ms(100);
	
	macRC522_CS_Disable();
	
}


static void RC522_SPI_Config ( void )
{
		stc_gpio_cfg_t stcGpioCfg;
		DDL_ZERO_STRUCT(stcGpioCfg);
    
    // 开启 GPIO 外设时钟 (HC32 特有)
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);;

    // 配置 CS, SCK, MOSI, RST 为推挽输出
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvH;

		
    Gpio_Init(RC522_MOSI_PORT, RC522_MOSI_PIN, &stcGpioCfg);
		Gpio_SetAfMode(RC522_MOSI_PORT, RC522_MOSI_PIN, GpioAf5);

    Gpio_Init(RC522_CS_PORT, RC522_CS_PIN, &stcGpioCfg);
		Gpio_SetAfMode(RC522_CS_PORT, RC522_CS_PIN, GpioAf2);
	
    Gpio_Init(RC522_SCK_PORT, RC522_SCK_PIN, &stcGpioCfg);
		Gpio_SetAfMode(RC522_SCK_PORT, RC522_SCK_PIN, GpioAf1);
		  
// 2. 配置输入引脚 (MISO)
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(RC522_MISO_PORT, RC522_MISO_PIN, &stcGpioCfg);
		Gpio_SetAfMode(RC522_MISO_PORT, RC522_MISO_PIN, GpioAf5);
		
		
		stcGpioCfg.enDir = GpioDirOut;
				    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
		Gpio_Init(RC522_RST_PORT, RC522_RST_PIN, &stcGpioCfg);

    // 3. 关键：解除 RC522 的复位状态
    // 先给个低脉冲复位一下，再拉高运行
    Gpio_WriteOutputIO(RC522_RST_PORT, RC522_RST_PIN, FALSE);
    delay1ms(100); 
    Gpio_WriteOutputIO(RC522_RST_PORT, RC522_RST_PIN, TRUE);
    delay1ms(200); 
    
    // 4. 设置初始电平
    //Gpio_WriteOutputIO(RC522_CS_PORT, RC522_CS_PIN, TRUE); // CS 默认高
    //Gpio_WriteOutputIO(RC522_SCK_PORT, RC522_SCK_PIN, FALSE); // SCK 默认低;	

		//Gpio_WriteOutputIO(RC522_RST_PORT, RC522_RST_PIN, TRUE);
	
}


