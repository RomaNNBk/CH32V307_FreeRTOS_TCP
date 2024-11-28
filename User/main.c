/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *task1 and task2 alternate printing
 */

/* FreeRTOS include. */
#include <FreeRTOS.h>
#include "task.h"

/* Standard includes. */
#include <signal.h>
#include <setjmp.h>
#include <time.h>

/* Test runner includes. */

/* System application includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_DHCP.h"
/*#include "iot_system_init.h" */

/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      1024
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;


/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0/1
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    while(1)
    {
        //printf("task1 entry\r\n");
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        vTaskDelay(250);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        vTaskDelay(250);
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{
	while(1)
	    {
	        //printf("task1 entry\r\n");
	        GPIO_SetBits(GPIOA, GPIO_Pin_0);
	        vTaskDelay(250);
	        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	        vTaskDelay(250);
	    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

#define FIRST_EXCEPTION_HANDLER        1

const uint8_t ucMACAddress[ 6 ] =
{
    configMAC_ADDR0,
    configMAC_ADDR1,
    configMAC_ADDR2,
    configMAC_ADDR3,
    configMAC_ADDR4,
    configMAC_ADDR5
};

/* The default IP and MAC address used by the demo.  The address configuration
 * defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
 * 1 but a DHCP server could not be contacted.  See the online documentation for
 * more information.  In both cases the node can be discovered using
 * "ping RTOSDemo". */
static const uint8_t ucIPAddress[ 4 ] =
{
    configIP_ADDR0,
    configIP_ADDR1,
    configIP_ADDR2,
    configIP_ADDR3
};
static const uint8_t ucNetMask[ 4 ] =
{
    configNET_MASK0,
    configNET_MASK1,
    configNET_MASK2,
    configNET_MASK3
};
static const uint8_t ucGatewayAddress[ 4 ] =
{
    configGATEWAY_ADDR0,
    configGATEWAY_ADDR1,
    configGATEWAY_ADDR2,
    configGATEWAY_ADDR3
};
static const uint8_t ucDNSServerAddress[ 4 ] =
{
    configDNS_SERVER_ADDR0,
    configDNS_SERVER_ADDR1,
    configDNS_SERVER_ADDR2,
    configDNS_SERVER_ADDR3
};

static void prvServerConnectionInstance( void *pvParameters )
{
int32_t lBytes, lSent, lTotalSent;
uint8_t cReceivedString[ 20 ];
Socket_t xConnectedSocket;
static const TickType_t xReceiveTimeOut = pdMS_TO_TICKS( 5000 );
static const TickType_t xSendTimeOut = pdMS_TO_TICKS( 5000 );
TickType_t xTimeOnShutdown;


	xConnectedSocket = ( Socket_t ) pvParameters;
	FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );
	FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof( xReceiveTimeOut ) );

	for( ;; )
	{
		/* Zero out the receive array so there is NULL at the end of the string
		when it is printed out. */
		memset( cReceivedString, 0x00, sizeof( cReceivedString ) );

		/* Receive data on the socket. */
		lBytes = FreeRTOS_recv( xConnectedSocket, cReceivedString, sizeof( cReceivedString ), 0 );
		uint8_t send[] = {0x01, 0x03, 0x02, 0x00, 0x37, 0xf9, 0x92};
		/* If data was received, echo it back. */
		if( lBytes >= 0 )
		{
			lSent = 0;
			lTotalSent = 0;

			/* Call send() until all the data has been sent. */
			while( ( lSent >= 0 ) && ( lTotalSent < lBytes ) )
			{
				lSent = FreeRTOS_send( xConnectedSocket, send, sizeof(send), 0 );
				lTotalSent += lSent;
			}

			if( lSent < 0 )
			{
				/* Socket closed? */
				break;
			}
		}
		else
		{
			/* Socket closed? */
			break;
		}
	}

	/* Initiate a shutdown in case it has not already been initiated. */
	FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );

	/* Wait for the shutdown to take effect, indicated by FreeRTOS_recv()
	returning an error. */
	xTimeOnShutdown = xTaskGetTickCount();
	do
	{
		if( FreeRTOS_recv( xConnectedSocket, cReceivedString, ipconfigTCP_MSS, 0 ) < 0 )
		{
			break;
		}
	} while( ( xTaskGetTickCount() - xTimeOnShutdown ) < 5000 );

	/* Finished with the socket and the task. */
	FreeRTOS_closesocket( xConnectedSocket );
	vTaskDelete( NULL );
}

static void prvConnectionListeningTask( void *pvParameters )
{
struct freertos_sockaddr xClient, xBindAddress;
Socket_t xListeningSocket, xConnectedSocket;
socklen_t xSize = sizeof( xClient );
static const TickType_t xReceiveTimeOut = portMAX_DELAY;
const BaseType_t xBacklog = 20;
WinProperties_t xWinProps;

	/* Just to prevent compiler warnings. */
	( void ) pvParameters;

	/* Attempt to open the socket. */
	xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );
	configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

	/* Set a time out so accept() will just wait for a connection. */
	FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );

	/* Fill in the buffer and window sizes that will be used by the socket. */
	xWinProps.lTxBufSize = 50;
	xWinProps.lTxWinSize = 3;
	xWinProps.lRxBufSize = 50;
	xWinProps.lRxWinSize = 3;

	/* Set the window and buffer sizes. */
	//FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_WIN_PROPERTIES, ( void * ) &xWinProps, sizeof( xWinProps ) );

	/* Bind the socket to the port that the client task will send to, then
	listen for incoming connections. */
	xBindAddress.sin_port = 10000;
	xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );
	FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );
	FreeRTOS_listen( xListeningSocket, xBacklog );


	for( ;; )
	{
		/* Wait for a client to connect. */
		xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
		configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

		/* Spawn a task to handle the connection. */
		xTaskCreate( prvServerConnectionInstance, "EchoServer", 256, ( void * ) xConnectedSocket, tskIDLE_PRIORITY, NULL );
	}
}
/*-----------------------------------------------------------*/

int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);

	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("FreeRTOS Kernel Version:%s\r\n",tskKERNEL_VERSION_NUMBER);

	GPIO_Toggle_INIT();


	FreeRTOS_IPInit(
	        ucIPAddress,
	        ucNetMask,
	        ucGatewayAddress,
	        ucDNSServerAddress,
	        ucMACAddress );

	/* create two task */
    xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )prvConnectionListeningTask,
                    (const char*    )"vCreateTCPServerSocket",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

	while(1)
	{
	    printf("shouldn't run at here!!\n");
	}
}
