
#include "stm32f10x.h"
#include "usart1.h"
#include "delay.h"
#include "stdio.h"
#include "Key.h"
#include "spi_flash.h"
#include "LED.h"

#include "gpio.h"
#include "delay.h"
#include "spi_radio.h"

#include "sx1280-hal.h"
#include "sx1280.h"
#include "radio.h"

#include "lcd.h"
#include "text.h"

/* 获取缓冲区的长度 */
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define  TxRxBufferSize (countof(Tx_Buffer)-1)

#define  FLASH_WriteAddress     0x00000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress
//#define  sFLASH_ID              0xEF3015     //W25X16
#define  sFLASH_ID              0xEF4015	 //W25Q16
     

/* 发送缓冲区初始化 */
uint8_t Tx_Buffer[] = " 感谢您\r\n";
uint8_t Rx_Buffer[TxRxBufferSize];


__IO uint32_t FlashID = 0;

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */

#if 1

/**************************************************************************************************************************************
Demo 程序流程  isMaster=true  为主机端，主机端发送一个"PING"数据LED闪烁后切换到接收，等待从机返回的应答"PONG"数据

               isMaster=false 为从机端，从机端接收到主机端发过来的"PING"数据后发送一个"PONG"数据作为应答，发送完"PONG"数据后LED闪烁
***************************************************************************************************************************************/

bool isMaster = false;//true;//主从选择


ModulationParams_t modulationParams;


  

/*!
 * Select mode of operation for the Ping Ping application
 */
//#define MODE_BLE
#define MODE_LORA
//#define MODE_GFSK
//#define MODE_FLRC


#define RF_BL_ADV_CHANNEL_38                        2478000000 // Hz

/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                RF_BL_ADV_CHANNEL_38 // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                             6//

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */

#define BUFFER_SIZE                                 58//52//56//48//32//20   //set >53
#define HEADED_LENGTH								4
#define SEND_LENGTH                                 (BUFFER_SIZE-8)//52//54
#define CHECK_SIZE									(BUFFER_SIZE-2)

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            10000 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            1000 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                                4


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );


/*!
 * \brief Define the possible message type for this application
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";


typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;


RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone
};


/*!
 * \brief The size of the buffer
 */
uint8_t BufferSize = BUFFER_SIZE;

/*!
 * \brief The buffer
 */
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief The State of the application
 */
AppStates_t AppState = APP_LOWPOWER;

#if defined( MODE_BLE )
/*!
 * \brief In case of BLE, the payload must contain the header
 */
typedef union
{
    struct BleAdvHeaderField_s
    {
        uint8_t pduType: 4;
        uint8_t rfu1:2;
        uint8_t txAddr:1;
        uint8_t rxAddr:1;
        uint8_t length:6;
        uint8_t rfu2:2;
    } Fields;
    uint8_t Serial[ 2 ];
}BleAdvHeaders_t;
BleAdvHeaders_t ble_header_adv;
#endif // MODE_BLE

PacketParams_t packetParams;

PacketStatus_t packetStatus;



void SysClock_48()
{ 
   RCC_PLLCmd(DISABLE);
   RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);//48M
   RCC_PLLCmd(ENABLE);
   while(!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void Tick_Configration()
{
    /* Setup SysTick Timer for 1ms interrupts ( not too often to save power ) */
    if( SysTick_Config( SystemCoreClock / 1000 ) )
    { 
        /* Capture error */ 
        while (1);
    }
}

void RCC_Configuration()
{   
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2|RCC_AHBPeriph_SRAM|RCC_AHBPeriph_SDIO, ENABLE);
  
  /* Enable peripheral Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2|RCC_APB1Periph_PWR, ENABLE);  
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_AFIO, ENABLE); 

}

void NVIC_Config()
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  #if 1
  EXTI_InitTypeDef  EXTI_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);

  EXTI_ClearITPendingBit(EXTI_Line10);
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  #endif
  /* Enable and set EXTI0 Interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;	 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void HW_Int()//MCU外围资源初始化
{
  SysClock_48(); 
  Tick_Configration();
  RCC_Configuration();
  GPIO_int();
  NVIC_Config();//设置模块DIO1脚连接的MCU IO口为外部上升沿中断,上数据收发完成后模块的DIO1脚会产生一个上升沿
  SPI2_Int();
  printf("\r\n HW init\r\n");
}

void LED_Indicate()//LED指示灯闪烁
{
 LED0_ON;
 Delay_Ms(20);
 LED0_OFF;
 Delay_Ms(20);
}



void SE243L_PA_Enable()//SE243L开PA通道
{
  GPIO_WriteBit(RADIO_CPS_PORT, RADIO_CPS_PIN, Bit_RESET);// CPS=0
  GPIO_WriteBit(RADIO_CSD_PORT, RADIO_CSD_PIN, Bit_SET);//CSD=1
  GPIO_WriteBit(RADIO_CTX_PORT, RADIO_CTX_PIN, Bit_SET);//CTX=1
  
  GPIO_WriteBit(ANT_SEL_PORT, ANT_SEL_PIN, Bit_SET);//选择SE2431L天线端口ANT2     SEL=1
//GPIO_WriteBit(ANT_SEL_PORT, ANT_SEL_PIN, Bit_RESET);//选择SE2431L天线端口ANT1   SEL=0
}

void SE243L_LNA_Enable()//SE243L开LAN通道
{
  GPIO_WriteBit(RADIO_CPS_PORT, RADIO_CPS_PIN, Bit_SET);//CPS=1
  GPIO_WriteBit(RADIO_CSD_PORT, RADIO_CSD_PIN, Bit_SET);//CSD=1
  GPIO_WriteBit(RADIO_CTX_PORT, RADIO_CTX_PIN, Bit_RESET);//CTX=0
  
  GPIO_WriteBit(ANT_SEL_PORT, ANT_SEL_PIN, Bit_SET);//选择SE2431L天线端口ANT2     SEL=1
//GPIO_WriteBit(ANT_SEL_PORT, ANT_SEL_PIN, Bit_RESET);//选择SE2431L天线端口ANT1   SEL=0
}

void SE243L_SLEEP()//SE243L进入SLEEP
{
  GPIO_WriteBit(RADIO_CPS_PORT, RADIO_CPS_PIN, Bit_RESET);//CPS=0
  GPIO_WriteBit(RADIO_CSD_PORT, RADIO_CSD_PIN, Bit_RESET);//CSD=0
  GPIO_WriteBit(RADIO_CTX_PORT, RADIO_CTX_PIN, Bit_RESET);//CTX=0
  
  GPIO_WriteBit(ANT_SEL_PORT, ANT_SEL_PIN, Bit_SET);
}


#endif

typedef struct _SEND_DATA_INFO
{
	uint32_t	length;		// image size length
	uint32_t  capacity;
	uint32_t  free;
	uint16_t	dirID;		//Dir ID
	uint16_t  fileID;     //File ID
	uint16_t  single;
	uint16_t  temp;
	uint16_t  creatTime[6];	//file creat date time : y/m/d/h/m/s
	uint8_t   battery;
	uint8_t   fileType;	// fileType
	uint8_t	  name[8]; 	//child name
	
} SEND_DATA_INFO;

#define SEND_DATA_INFO_SIZE		42

typedef struct _SEND_DATA_BUFFER
{
	uint8_t	header;		// send "@",ack "#"
	uint8_t	host;		//host ID
	uint16_t  child;      //child ID
	uint8_t	data[SEND_LENGTH]; //image data pack
	uint16_t  index;		// packet index
	uint16_t  checknum;       // 
} SEND_DATA_BUFFER;


SEND_DATA_BUFFER ReSendTab[1];
SEND_DATA_BUFFER Host2Child[16];

uint8_t	HostID=0x31;
uint8_t	HostName[8]="CLH1";
uint16_t  ChildID=0x4331;
uint8_t	ChildName[8]="CLC1";
uint16_t	ChildIDTab[16];
uint8_t   g_childIndex=0;
uint8_t   g_lastchildIndex=0xff;
uint8_t   g_childSum = 3;


uint16_t GetCrc16(uint8_t *ptr, uint32_t count)
{
	uint16_t crc, i;

	crc = 0;
	while (count--)
	{
		crc = crc ^ (int) *ptr++ << 8;//从packet_data中取一个字节数据，强转为16为int，再把低八位移到高八位，赋值给crc

		for (i = 0; i < 8; i++)
		{
			if (crc & 0x8000)//判断数据的最高位数据是否为1
				crc = crc << 1 ^ 0x1021; // CRC-ITU
			else
				crc = crc << 1;
		}
	}

	return (crc & 0xFFFF);
}

void lora_reset(void)
{

		HW_Int();
	
		Radio.Init( &Callbacks );
		Radio.SetRegulatorMode( USE_DCDC ); // Can also be set in LDO mode but consume more power
		
		modulationParams.PacketType = PACKET_TYPE_LORA;
		modulationParams.Params.LoRa.SpreadingFactor = LORA_SF6;// Effective Data Rate 50.78Kb/s
		modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;
		modulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;
		
		packetParams.PacketType = PACKET_TYPE_LORA;
		packetParams.Params.LoRa.PreambleLength = 12;
		packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
		packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
		packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
		packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	
		
		Radio.SetStandby( STDBY_RC );
		Radio.SetPacketType( modulationParams.PacketType );
		Radio.SetModulationParams( &modulationParams );
		Radio.SetPacketParams( &packetParams );
		Radio.SetRfFrequency( RF_FREQUENCY );//频点设置
		Radio.SetBufferBaseAddresses( 0x00, 0x00 );
		Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_02_US );//功率设置
		
	   // Radio.SetInterruptMode();
		Radio.SetPollingMode();  
	  
	  AppState = APP_LOWPOWER;
	  printf("\r\nVersion: %04x",Radio.GetFirmwareVersion());

}

void lora_running(void)
{
	uint32_t index;

	for(index = 0;index < g_childSum;index++)
	{
		ChildIDTab[index] = 0x4331+index;
		memset((uint8_t*)&Host2Child[index],0x00,BUFFER_SIZE);
		Host2Child[index].header = 0xBB;
		Host2Child[index].host = HostID;
		Host2Child[index].child = ChildIDTab[index];
		Host2Child[index].index = 0xfefe;
		sprintf(Host2Child[index].data,"@@$$%04x$%s$%02x$%s$",ChildIDTab[index],ChildName,HostID,HostName);

		Host2Child[index].checknum = GetCrc16((uint8_t*)&Host2Child[index],CHECK_SIZE);
	}

	lora_reset();
	
	//printf("\r\nTx\r\n");
	//for(index = 0;index < BUFFER_SIZE;index++)
	//	printf("%02x ",*((uint8_t*)&Host2Child[0]+index));

	SE243L_PA_Enable();//切换到PA通道�
	Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
	Radio.SendPayload( (uint8_t*)&Host2Child[g_childIndex], BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
	AppState = APP_LOWPOWER;

}

int main(void)
{	
  unsigned char value;
  unsigned char GetFlag;
  uint32_t index;
	USART1_Config();
	Key_GPIO_Config();	
	LED_GPIO_Config();
	
	POINT_COLOR=RED;

	LCD_Init();
	LCD_Clear(WHITE);
	LCD_ShowString(30,50,"Mini STM32 ^_^",POINT_COLOR);	
	LCD_ShowString(30,70,"TFTLCD TEST",POINT_COLOR);	
	LCD_ShowString(30,90,"Pioneer",POINT_COLOR);
	LCD_ShowString(30,110,"2009/12/30",POINT_COLOR);	
	
	printf("\r\n uart1 init\r\n");
	SPI_FLASH_Init();

	FlashID = SPI_FLASH_ReadID();
	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF3015 */
	{
		printf("\r\n flash W25Q16 !\r\n");
	}
	
	LED1_OFF;
	LED0_ON;
	LED2_OFF;
	
	lora_running();
	
	while(1)
	{
	#if 0
		 value=KEY_Scan();//得到键值
		LED_Indicate();//LED闪烁
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF3015 */
		{
			if(value==1)
			{
				/* Erase SPI FLASH Sector to write on */
				SPI_FLASH_SectorErase(FLASH_SectorToErase);	
				SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, TxRxBufferSize);
				printf("\r\n 写入的数据为：%s \r\t", Tx_Buffer);
			}
			if(value==2)
			{
				SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, TxRxBufferSize);
				printf("\r\n 读出的数据为：%s \r\n", Tx_Buffer);
			}
    	}
		#endif
	#if 1
	 
        SX1280ProcessIrqs( );//当发送完一包数据或接收到一包数据后模块的DIO1脚产生一个上升沿，
        
        switch( AppState )
        {
            case APP_RX:
                AppState = APP_LOWPOWER;
                GetFlag = Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
				if(GetFlag == 1)
				{
					AppState = APP_RX_TIMEOUT;
					printf("\r\nError Getpayload Size over maxlength \r\n");
					break;
				}
				if(BufferSize != BUFFER_SIZE)
				{
					AppState = APP_RX_TIMEOUT;
					printf("\r\nError Getpayload Size %d != %d \r\n",BufferSize,BUFFER_SIZE);
					break;
				}

				SE243L_PA_Enable();//切换到PA通道
				Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				Radio.SendPayload( (uint8_t*)&Host2Child[g_childIndex], BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );

				//printf("\r\nRx\r\n");
				//for(index = 0;index < BUFFER_SIZE;index++)
				//	printf("%02x ",*((uint8_t*)Buffer+index));

                break;

            case APP_TX:
                AppState = APP_LOWPOWER;
				
				SE243L_LNA_Enable();//切换到LNA通道
				Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );

                break;

            case APP_RX_TIMEOUT:
                 AppState = APP_LOWPOWER;
                 Delay_Ms(200);
				 LED_Indicate();
				 SE243L_PA_Enable();//切换到PA通道
				 Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				 Radio.SendPayload( (uint8_t*)&Host2Child[g_childIndex], BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
               
                break;

            case APP_RX_ERROR:
                AppState = APP_LOWPOWER;

				SE243L_PA_Enable();//切换到PA通道
				Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				Radio.SendPayload( (uint8_t*)&Host2Child[g_childIndex], BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );

                break;

            case APP_TX_TIMEOUT:
                
                AppState = APP_LOWPOWER;
						    
				//SE243L_LNA_Enable();//切换到LNA通道
                //Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                //Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } ); 
                
				SE243L_PA_Enable();//切换到PA通道
				Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				Radio.SendPayload( (uint8_t*)&Host2Child[g_childIndex], BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                break;

            case APP_LOWPOWER:
                break;

            default:
                // Set low power
                break;
        }
	#endif
	}
}



uint32_t Rxcount=0;
void OnTxDone( void )
{
  
 AppState = APP_TX;
  //printf("\r\n OnTxDone\r\n");
}

void OnRxDone( void )
{
	Rxcount++;
    AppState = APP_RX;
	printf("\r\n OnRxDone %d\r\n",Rxcount);
}

void OnTxTimeout( void )
{
    AppState = APP_TX_TIMEOUT;
	printf("\r\n OnTxTimeout\r\n");

}

void OnRxTimeout( void )
{
    AppState = APP_RX_TIMEOUT;
	printf("\r\n OnRxTimeout\r\n");
}

void OnRxError( IrqErrorCode_t errorCode )
{
    AppState = APP_RX_ERROR;
	printf("\r\n OnRxError\r\n");
}

void OnRangingDone( IrqRangingCode_t val )
{
  
}

void OnCadDone( bool channelActivityDetected )
{
  
}




